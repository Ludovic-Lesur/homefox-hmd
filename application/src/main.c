/*
 * main.c
 *
 *  Created on: 13 sep. 2025
 *      Author: Ludo
 */

// Drivers.
#include "exti.h"
#include "gpio.h"
#include "i2c_address.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Utils.
#include "error.h"
#include "types.h"
// Components.
#include "led.h"
#include "sensors_hw.h"
// Middleware
#include "cli.h"
#include "sigfox_ep_flags.h"
#include "sigfox_ep_api.h"
#include "sigfox_rc.h"
// Applicative.
#include "error_base.h"
#include "hmd_flags.h"

/*** MAIN local macros ***/

// Sigfox payload lengths.
#define HMD_SIGFOX_STARTUP_DATA_SIZE                8
#define HMD_SIGFOX_ERROR_STACK_DATA_SIZE            12
#define HMD_SIGFOX_MONITORING_DATA_SIZE             6
#ifdef HMD_ENS16X_ENABLE
#define HMD_SIGFOX_AIR_QUALITY_DATA_SIZE            7
#define HMD_ENS16X_ACQUISITION_DELAY_MS             10000
#define HMD_ENS16X_ACQUISITION_TIMEOUT_MS           300000
#ifdef ENS16X_DRIVER_DEVICE_ENS161
#define HMD_ENS16X_ACQUISITION_MODE                 ENS16X_SENSING_MODE_LOW_POWER
#else
#define HMD_ENS16X_ACQUISITION_MODE                 ENS16X_SENSING_MODE_STANDARD
#endif
#endif
#ifdef HMD_FXLS89XXXX_ENABLE
#define HMD_SIGFOX_ACCELEROMETER_EVENT_DATA_SIZE    1
#endif
// Error stack message period.
#define HMD_ERROR_STACK_PERIOD_SECONDS              86400
// Voltage hysteresis for radio.
#define HMD_RADIO_ON_VSTR_THRESHOLD_MV              3700
#define HMD_RADIO_OFF_VSTR_THRESHOLD_MV             3500
// Error values.
#define HMD_ERROR_VALUE_ANALOG_16BITS               0xFFFF
#define HMD_ERROR_VALUE_TEMPERATURE                 0x7FF
#define HMD_ERROR_VALUE_HUMIDITY                    0xFF
#define HMD_ERROR_VALUE_TVOC                        0xFFFF
#define HMD_ERROR_VALUE_ECO2                        0xFFFF
#define HMD_ERROR_VALUE_AQI_UBA                     0b1111
#define HMD_ERROR_VALUE_AQI_S                       0xFFF

/*** MAIN local structures ***/

/*******************************************************************/
typedef enum {
    HMD_STATE_STARTUP,
    HMD_STATE_WAKEUP,
    HMD_STATE_BUTTON,
    HMD_STATE_MEASURE,
    HMD_STATE_MONITORING_DATA,
#ifdef HMD_ENS16X_ENABLE
    HMD_STATE_AIR_QUALITY_DATA,
#endif
#ifdef HMD_FXLS89XXXX_ENABLE
    HMD_STATE_ACCELEROMETER_DATA,
#endif
    HMD_STATE_ERROR_STACK,
    HMD_STATE_SLEEP,
    HMD_STATE_LAST
} HMD_state_t;

/*******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned unused :2;
        unsigned lse_status :1;
        unsigned lsi_status :1;
        unsigned fxls89xxxx_enable :1;
        unsigned ens16x_enable :1;
        unsigned ens21x_enable :1;
        unsigned sht3x_enable :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_status_t;

/*******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned radio_enabled : 1;
        unsigned error_stack_request :1;
        unsigned accelerometer_request :1;
        unsigned button_request :1;
        unsigned monitoring_request :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_flags_t;

/*******************************************************************/
typedef union {
    uint8_t frame[HMD_SIGFOX_STARTUP_DATA_SIZE];
    struct {
        unsigned reset_reason :8;
        unsigned major_version :8;
        unsigned minor_version :8;
        unsigned commit_index :8;
        unsigned commit_id :28;
        unsigned dirty_flag :4;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_sigfox_startup_data_t;

/*******************************************************************/
typedef union {
    uint8_t frame[HMD_SIGFOX_MONITORING_DATA_SIZE];
    struct {
        unsigned vbatt_mv :16;
        unsigned unused :4;
        unsigned tamb_tenth_degrees :12;
        unsigned hamb_percent :8;
        unsigned status :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_sigfox_monitoring_data_t;

#ifdef HMD_ENS16X_ENABLE
/*******************************************************************/
typedef union {
    uint8_t frame[HMD_SIGFOX_AIR_QUALITY_DATA_SIZE];
    struct {
        unsigned acquisition_duration_tens_seconds :6;
        unsigned acquisition_status :2;
        unsigned tvoc_ppb :16;
        unsigned eco2_ppm :16;
        unsigned aqi_uba :4;
        unsigned aqi_s :12;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_sigfox_air_quality_data_t;
#endif

#ifdef HMD_FXLS89XXXX_ENABLE
/*******************************************************************/
typedef union {
    uint8_t frame[HMD_SIGFOX_ACCELEROMETER_EVENT_DATA_SIZE];
    struct {
        unsigned event_source :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_sigfox_accelerometer_data_t;
#endif

/*******************************************************************/
typedef struct {
    // State machine.
    HMD_state_t state;
    HMD_status_t status;
    volatile HMD_flags_t flags;
    uint32_t monitoring_next_time_seconds;
    uint32_t error_stack_next_time_seconds;
} HMD_context_t;

/*** MAIN local global variables ***/

#ifndef HMD_MODE_CLI
static HMD_context_t hmd_ctx;
#endif

/*** MAIN local functions ***/

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_button_irq_callback(void) {
    hmd_ctx.flags.button_request = 1;
}
#endif

#if (!(defined HMD_MODE_CLI) && (defined HMD_FXLS89XXXX_ENABLE))
/*******************************************************************/
static void _HMD_motion_irq_callback(void) {
    hmd_ctx.flags.accelerometer_request = 1;
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_init_context(void) {
    // Init context.
    hmd_ctx.state = HMD_STATE_STARTUP;
    hmd_ctx.flags.all = 0;
    hmd_ctx.flags.radio_enabled = 1;
    hmd_ctx.flags.error_stack_request = 1;
    hmd_ctx.monitoring_next_time_seconds = HMD_MONITORING_PERIOD_SECONDS;
    hmd_ctx.error_stack_next_time_seconds = HMD_ERROR_STACK_PERIOD_SECONDS;
    hmd_ctx.status.all = 0;
#ifdef HMD_SHT3X_ENABLE
    hmd_ctx.status.sht3x_enable = 1;
#endif
#ifdef HMD_ENS21X_ENABLE
    hmd_ctx.status.ens21x_enable = 1;
#endif
#ifdef HMD_ENS16X_ENABLE
    hmd_ctx.status.ens16x_enable = 1;
#endif
#ifdef HMD_FXLS89XXXX_ENABLE
    hmd_ctx.status.fxls89xxxx_enable = 1;
    // Set motion interrupt callback address.
    SENSORS_HW_set_accelerometer_irq_callback(&_HMD_motion_irq_callback);
#endif
}
#endif

/*******************************************************************/
static void _HMD_init_hw(void) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
#ifndef HMD_MODE_DEBUG
    IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
#ifndef HMD_MODE_CLI
    BUTTON_status_t button_status = BUTTON_SUCCESS;
#endif
    // Init error stack
    ERROR_stack_init();
    // Init memory.
    NVIC_init();
    // Init power module and clock tree.
    PWR_init();
    rcc_status = RCC_init(NVIC_PRIORITY_CLOCK);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init GPIOs.
    GPIO_init();
    POWER_init();
    EXTI_init();
    // Start independent watchdog.
#ifndef HMD_MODE_DEBUG
    iwdg_status = IWDG_init();
    IWDG_stack_error(ERROR_BASE_IWDG);
    IWDG_reload();
#endif
    // High speed oscillator.
    rcc_status = RCC_switch_to_hsi();
    RCC_stack_error(ERROR_BASE_RCC);
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init RTC.
    rtc_status = RTC_init(NULL, NVIC_PRIORITY_RTC);
    RTC_stack_error(ERROR_BASE_RTC);
    // Init delay timer.
    lptim_status = LPTIM_init(NVIC_PRIORITY_DELAY);
    LPTIM_stack_error(ERROR_BASE_LPTIM);
    // Init HMI.
#ifndef HMD_MODE_CLI
    button_status = BUTTON_init(&_HMD_button_irq_callback);
    BUTTON_stack_error(ERROR_BASE_BUTTON);
#endif
    led_status = LED_init();
    LED_stack_error(ERROR_BASE_LED);
}

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_send_sigfox_message(SIGFOX_EP_API_application_message_t* application_message) {
    // Local variables.
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    uint8_t status = 0;
    // Directly exit of the radio is disabled due to low battery voltage.
    if (hmd_ctx.flags.radio_enabled == 0) goto errors;
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_check_status(0);
    // Send message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(application_message);
    SIGFOX_EP_API_check_status(0);
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_check_status(0);
    goto end;
errors:
    SIGFOX_EP_API_close();
    UNUSED(status);
end:
    return;
}
#endif

/*** MAIN functions ***/

#ifndef HMD_MODE_CLI
/*******************************************************************/
int main(void) {
    // Init board.
    _HMD_init_context();
    _HMD_init_hw();
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    MATH_status_t math_status = MATH_SUCCESS;
    ERROR_code_t error_code = 0;
    HMD_sigfox_startup_data_t sigfox_startup_data;
    HMD_sigfox_monitoring_data_t sigfox_monitoring_data;
#ifdef HMD_SHT3X_ENABLE
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
#endif
#ifdef HMD_ENS21X_ENABLE
    ENS21X_status_t ens21x_status = ENS21X_SUCCESS;
#endif
#ifdef HMD_ENS16X_ENABLE
    ENS16X_status_t ens16x_status = ENS16X_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    ENS16X_device_status_t ens16x_device_status;
    uint32_t ens16x_acquisition_time_ms = 0;
    ENS16X_air_quality_data_t air_quality_data;
    HMD_sigfox_air_quality_data_t sigfox_air_quality_data;
#endif
#ifdef HMD_FXLS89XXXX_ENABLE
    HMD_sigfox_accelerometer_data_t sigfox_accelerometer_data;
#endif
    SIGFOX_EP_API_application_message_t application_message;
    uint8_t sigfox_error_stack_data[HMD_SIGFOX_ERROR_STACK_DATA_SIZE];
    uint8_t idx = 0;
    int32_t generic_s32_1 = 0;
    int32_t generic_s32_2 = 0;
    uint8_t generic_u8;
    uint32_t generic_u32 = 0;
    // Application message default parameters.
    application_message.common_parameters.number_of_frames = 3;
    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    application_message.bidirectional_flag = 0;
#endif
    application_message.ul_payload = SIGFOX_NULL;
    application_message.ul_payload_size_bytes = 0;
    // Main loop.
    while (1) {
        // Perform state machine.
        switch (hmd_ctx.state) {
        case HMD_STATE_STARTUP:
            IWDG_reload();
            // Fill reset reason and software version.
            sigfox_startup_data.reset_reason = PWR_get_reset_flags();
            sigfox_startup_data.major_version = GIT_MAJOR_VERSION;
            sigfox_startup_data.minor_version = GIT_MINOR_VERSION;
            sigfox_startup_data.commit_index = GIT_COMMIT_INDEX;
            sigfox_startup_data.commit_id = GIT_COMMIT_ID;
            sigfox_startup_data.dirty_flag = GIT_DIRTY_FLAG;
            // Clear reset flags.
            PWR_clear_reset_flags();
            // Send SW version frame.
            application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
            application_message.ul_payload = (sfx_u8*) (sigfox_startup_data.frame);
            application_message.ul_payload_size_bytes = HMD_SIGFOX_STARTUP_DATA_SIZE;
            _HMD_send_sigfox_message(&application_message);
            // Compute next state.
            hmd_ctx.state = HMD_STATE_ERROR_STACK;
            break;
        case HMD_STATE_WAKEUP:
            IWDG_reload();
            // Calibrate clocks.
            rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
            RCC_stack_error(ERROR_BASE_RCC);
            // Compute next state.
            if (hmd_ctx.flags.accelerometer_request != 0) {
                // Clear flag and update state.
                hmd_ctx.flags.accelerometer_request = 0;
#ifdef HMD_FXLS89XXXX_ENABLE
                hmd_ctx.state = HMD_STATE_ACCELEROMETER_DATA;
#else
                hmd_ctx.state = HMD_STATE_SLEEP;
#endif
            }
            else if (hmd_ctx.flags.button_request != 0) {
                // Clear flag and update state.
                hmd_ctx.flags.button_request = 0;
                hmd_ctx.state = HMD_STATE_BUTTON;
            }
            else if (hmd_ctx.flags.monitoring_request != 0) {
                // Clear flag and update state.
                hmd_ctx.flags.monitoring_request = 0;
                hmd_ctx.state = HMD_STATE_MEASURE;
            }
            else {
                hmd_ctx.state = HMD_STATE_SLEEP;
            }
            break;
        case HMD_STATE_BUTTON:
            IWDG_reload();
            // TODO
            // Compute next state.
            hmd_ctx.state = HMD_STATE_SLEEP;
            break;
        case HMD_STATE_MEASURE:
            IWDG_reload();
            // Reset frames.
            sigfox_monitoring_data.status = 0;
            sigfox_monitoring_data.unused = 0;
            sigfox_monitoring_data.vbatt_mv = HMD_ERROR_VALUE_ANALOG_16BITS;
            sigfox_monitoring_data.tamb_tenth_degrees = HMD_ERROR_VALUE_TEMPERATURE;
            sigfox_monitoring_data.hamb_percent = HMD_ERROR_VALUE_HUMIDITY;
#ifdef HMD_ENS16X_ENABLE
            sigfox_air_quality_data.acquisition_duration_tens_seconds = 0;
            sigfox_air_quality_data.acquisition_status = ENS16X_VALIDITY_FLAG_INVALID_OUTPUT;
            sigfox_air_quality_data.tvoc_ppb = HMD_ERROR_VALUE_TVOC;
            sigfox_air_quality_data.eco2_ppm = HMD_ERROR_VALUE_ECO2;
            sigfox_air_quality_data.aqi_uba = HMD_ERROR_VALUE_AQI_UBA;
            sigfox_air_quality_data.aqi_s = HMD_ERROR_VALUE_AQI_S;
#endif
            // Turn sensors on.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
#ifdef HMD_ENS21X_ENABLE
            // Get temperature and humidity from ENS210.
            ens21x_status = ENS21X_get_temperature_humidity(I2C_ADDRESS_ENS210, &generic_s32_1, &generic_s32_2);
            ENS21X_stack_error(ERROR_BASE_ENS210);
            // Check status.
            if (ens21x_status == ENS21X_SUCCESS) {
               // Convert temperature.
               math_status = MATH_integer_to_signed_magnitude(generic_s32_1, 11, &generic_u32);
               MATH_stack_error(ERROR_BASE_MATH);
               if (math_status == MATH_SUCCESS) {
                   sigfox_monitoring_data.tamb_tenth_degrees = (uint8_t) generic_u32;
               }
               sigfox_monitoring_data.hamb_percent = (uint8_t) generic_s32_2;
            }
#endif
#ifdef HMD_SHT3X_ENABLE
            if ((sigfox_monitoring_data.tamb_tenth_degrees == HMD_ERROR_VALUE_TEMPERATURE) || (sigfox_monitoring_data.hamb_percent == HMD_ERROR_VALUE_HUMIDITY)) {
                // Get temperature and humidity from SHT30.
                sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30, &generic_s32_1, &generic_s32_2);
                SHT3X_stack_error(ERROR_BASE_SHT30);
                // Check status.
                if (sht3x_status == SHT3X_SUCCESS) {
                   // Convert temperature.
                   math_status = MATH_integer_to_signed_magnitude(generic_s32_1, 11, &generic_u32);
                   MATH_stack_error(ERROR_BASE_MATH);
                   if (math_status == MATH_SUCCESS) {
                       sigfox_monitoring_data.tamb_tenth_degrees = (uint8_t) generic_u32;
                   }
                   sigfox_monitoring_data.hamb_percent = (uint8_t) generic_s32_2;
                }
            }
#endif
#ifdef HMD_ENS16X_ENABLE
            if ((sigfox_monitoring_data.tamb_tenth_degrees != HMD_ERROR_VALUE_TEMPERATURE) && (sigfox_monitoring_data.hamb_percent != HMD_ERROR_VALUE_HUMIDITY)) {
                // Reset duration.
                ens16x_acquisition_time_ms = 0;
                // Start acquisition.
                ens16x_status = ENS16X_start_acquisition(I2C_ADDRESS_ENS16X, HMD_ENS16X_ACQUISITION_MODE, sigfox_monitoring_data.tamb_tenth_degrees, sigfox_monitoring_data.hamb_percent);
                ENS16X_stack_error(ERROR_BASE_ENS16X);
                // Directly exit in case of error.
                if (ens16x_status == ENS16X_SUCCESS) {
                    // Wait for acquisition time.
                    do {
                        // Reload watchdog.
                        IWDG_reload();
                        // Low power delay.
                        lptim_status = LPTIM_delay_milliseconds(HMD_ENS16X_ACQUISITION_DELAY_MS, LPTIM_DELAY_MODE_STOP);
                        LPTIM_stack_error(ERROR_BASE_LPTIM);
                        // Update duration.
                        ens16x_acquisition_time_ms += HMD_ENS16X_ACQUISITION_DELAY_MS;
                        // Read device status.
                        ens16x_status = ENS16X_get_device_status(I2C_ADDRESS_ENS16X, &ens16x_device_status);
                        ENS16X_stack_error(ERROR_BASE_ENS16X);
                        // Check status.
                        if (ens16x_status != ENS16X_SUCCESS) break;
                        // Check data validity.
                        if (ens16x_device_status.validity_flag == ENS16X_VALIDITY_FLAG_NORMAL_OPERATION) {
                            // Read data.
                            ens16x_status = ENS16X_read_air_quality(I2C_ADDRESS_ENS16X, &air_quality_data);
                            ENS16X_stack_error(ERROR_BASE_ENS16X);
                            // Check status.
                            if (ens16x_status == ENS16X_SUCCESS) {
                                sigfox_air_quality_data.tvoc_ppb = (air_quality_data.tvoc_ppb);
                                sigfox_air_quality_data.eco2_ppm = (air_quality_data.eco2_ppm);
                                sigfox_air_quality_data.aqi_uba = (air_quality_data.aqi_uba);
#ifdef ENS16X_DRIVER_DEVICE_ENS161
                                sigfox_air_quality_data.aqi_s = (air_quality_data.aqi_s);
#endif
                            }
                            break;
                        }
                    }
                    while (ens16x_acquisition_time_ms < HMD_ENS16X_ACQUISITION_TIMEOUT_MS);
                    // Stop acquisition.
                    ens16x_status = ENS16X_stop_acquisition(I2C_ADDRESS_ENS16X);
                    ENS16X_stack_error(ERROR_BASE_ENS16X);
                    // Update status.
                    sigfox_air_quality_data.acquisition_duration_tens_seconds = (ens16x_acquisition_time_ms / 10000);
                    sigfox_air_quality_data.acquisition_status = ens16x_device_status.validity_flag;
                }
            }
#endif
            // Turn sensors off.
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
            // Get voltages measurements.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VBATT_MV, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
            if (analog_status == ANALOG_SUCCESS) {
                // Update data.
                sigfox_monitoring_data.vbatt_mv = (uint16_t) generic_s32_1;
                // Voltage hysteresis for radio.
                if (generic_s32_1 < HMD_RADIO_OFF_VSTR_THRESHOLD_MV) {
                    hmd_ctx.flags.radio_enabled = 0;
                }
                if (generic_s32_1 > HMD_RADIO_ON_VSTR_THRESHOLD_MV) {
                    hmd_ctx.flags.radio_enabled = 1;
                }

            }
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG);
            // Get clock status.
            rcc_status = RCC_get_status(RCC_CLOCK_LSI, &generic_u8);
            RCC_stack_error(ERROR_BASE_RCC);
            hmd_ctx.status.lsi_status = (generic_u8 == 0) ? 0b0 : 0b1;
            rcc_status = RCC_get_status(RCC_CLOCK_LSE, &generic_u8);
            RCC_stack_error(ERROR_BASE_RCC);
            hmd_ctx.status.lse_status = (generic_u8 == 0) ? 0b0 : 0b1;
            // Update status.
            sigfox_monitoring_data.status = hmd_ctx.status.all;
            // Compute next state.
            hmd_ctx.state = HMD_STATE_MONITORING_DATA;
            break;
        case HMD_STATE_MONITORING_DATA:
            IWDG_reload();
            // Send uplink monitoring frame.
            application_message.ul_payload = (sfx_u8*) (sigfox_monitoring_data.frame);
            application_message.ul_payload_size_bytes = HMD_SIGFOX_MONITORING_DATA_SIZE;
            _HMD_send_sigfox_message(&application_message);
            // Compute next state.
#ifdef HMD_ENS16X_ENABLE
            hmd_ctx.state = HMD_STATE_AIR_QUALITY_DATA;
#else
            hmd_ctx.state = HMD_STATE_ERROR_STACK;
#endif
            break;
#ifdef HMD_ENS16X_ENABLE
        case HMD_STATE_AIR_QUALITY_DATA:
            // Send uplink monitoring frame.
            application_message.ul_payload = (sfx_u8*) (sigfox_air_quality_data.frame);
            application_message.ul_payload_size_bytes = HMD_SIGFOX_AIR_QUALITY_DATA_SIZE;
            _HMD_send_sigfox_message(&application_message);
            // Compute next state.
            hmd_ctx.state = HMD_STATE_ERROR_STACK;
            break;
#endif
#ifdef HMD_FXLS89XXXX_ENABLE
        case HMD_STATE_ACCELEROMETER_DATA:
            IWDG_reload();
            // TODO
            // Compute next state.
            hmd_ctx.state = HMD_STATE_SLEEP;
            break;
#endif
        case HMD_STATE_ERROR_STACK:
            IWDG_reload();
            // Check period.
            if (hmd_ctx.flags.error_stack_request != 0) {
                // Clear flag.
                hmd_ctx.flags.error_stack_request = 0;
                // Import Sigfox library error stack.
                ERROR_import_sigfox_stack();
                // Check stack.
                if (ERROR_stack_is_empty() == 0) {
                    // Read error stack.
                    for (idx = 0; idx < (HMD_SIGFOX_ERROR_STACK_DATA_SIZE >> 1); idx++) {
                        error_code = ERROR_stack_read();
                        sigfox_error_stack_data[(idx << 1) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
                        sigfox_error_stack_data[(idx << 1) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
                    }
                    // Send error stack frame.
                    application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
                    application_message.ul_payload = (sfx_u8*) (sigfox_error_stack_data);
                    application_message.ul_payload_size_bytes = HMD_SIGFOX_ERROR_STACK_DATA_SIZE;
                    _HMD_send_sigfox_message(&application_message);
                }
            }
            // Compute next state.
            hmd_ctx.state = HMD_STATE_SLEEP;
            break;
        case HMD_STATE_SLEEP:
            // Enter stop mode.
            IWDG_reload();
            PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP);
            IWDG_reload();
            // Read uptime.
            generic_u32 = RTC_get_uptime_seconds();
            // Periodic monitoring.
            if (generic_u32 >= hmd_ctx.monitoring_next_time_seconds) {
               // Set periodic request and compute next time.
               hmd_ctx.flags.monitoring_request = 1;
               hmd_ctx.monitoring_next_time_seconds = (generic_u32 + HMD_MONITORING_PERIOD_SECONDS);
            }
            if (generic_u32 >= hmd_ctx.error_stack_next_time_seconds) {
               // Set periodic request and compute next time.
               hmd_ctx.flags.error_stack_request = 1;
               hmd_ctx.error_stack_next_time_seconds = (generic_u32 + HMD_ERROR_STACK_PERIOD_SECONDS);
            }
            // Check wake-up flags.
            if ((hmd_ctx.flags.monitoring_request != 0) || (hmd_ctx.flags.button_request != 0) || (hmd_ctx.flags.accelerometer_request != 0)) {
                // Turn device on.
                hmd_ctx.state = HMD_STATE_WAKEUP;
            }
            break;
        default:
            // Unknown state.
            hmd_ctx.state = HMD_STATE_SLEEP;
            break;
        }
    }
    return 0;
}
#endif

#ifdef HMD_MODE_CLI
/*******************************************************************/
int main(void) {
    // Local variables.
    CLI_status_t cli_status = CLI_SUCCESS;
    // Init board.
    _HMD_init_hw();
    // Init command line interface.
    cli_status = CLI_init();
    CLI_stack_error(ERROR_BASE_CLI);
    // Main loop.
    while (1) {
        // Enter sleep mode.
        IWDG_reload();
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        IWDG_reload();
        // Process command line interface.
        cli_status = CLI_process();
        CLI_stack_error(ERROR_BASE_CLI);
    }
    return 0;
}
#endif
