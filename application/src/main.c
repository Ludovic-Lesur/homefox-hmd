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
#include "mcu_mapping.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "nvm.h"
#include "nvm_address.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Utils.
#include "error.h"
#include "types.h"
// Components.
#include "fxls89xxxx.h"
#include "fxls89xxxx_configuration.h"
#include "led.h"
#include "sensors_hw.h"
// Middleware
#include "cli.h"
#include "sigfox_ep_flags.h"
#include "sigfox_ep_frames.h"
#include "sigfox_ep_api.h"
#include "sigfox_rc.h"
// Applicative.
#include "error_base.h"
#include "hmd_flags.h"

/*** MAIN local macros ***/

// Monitoring period.
#define HMD_MONITORING_PERIOD_MINUTES_DEFAULT               60
#define HMD_MONITORING_PERIOD_MINUTES_MIN                   10
#define HMD_MONITORING_PERIOD_MINUTES_MAX                   10080
// Downlink period.
#define HMD_DOWNLINK_PERIOD_SECONDS                         86400
// Error stack.
#define HMD_ERROR_STACK_BLANKING_TIME_SECONDS               86400
// Voltage hysteresis for radio.
#define HMD_RADIO_ON_VSTR_THRESHOLD_MV                      3700
#define HMD_RADIO_OFF_VSTR_THRESHOLD_MV                     3500
// VBATT indicator.
#define HMD_VBATT_INDICATOR_RANGE                           7
#define HMD_VBATT_INDICATOR_DELAY_MS                        3000
// Air quality.
#define HMD_AIR_QUALITY_PERIOD_MINUTES_DEFAULT              60
#define HMD_AIR_QUALITY_PERIOD_MINUTES_MIN                  10
#define HMD_AIR_QUALITY_PERIOD_MINUTES_MAX                  10080
#define HMD_AIR_QUALITY_ACQUISITION_DELAY_MS                10000
#define HMD_AIR_QUALITY_ACQUISITION_LED_BLINK_MS            100
#define HMD_AIR_QUALITY_ACQUISITION_TIME_MIN_MS             120000
#define HMD_AIR_QUALITY_ACQUISITION_TIME_MAX_MS             360000
#ifdef ENS16X_DRIVER_DEVICE_ENS161
#define HMD_AIR_QUALITY_ACQUISITION_MODE                    ENS16X_OPERATING_MODE_LOW_POWER
#else
#define HMD_AIR_QUALITY_ACQUISITION_MODE                    ENS16X_OPERATING_MODE_STANDARD
#endif
// Accelerometer.
#define HMD_ACCELEROMETER_BLANKING_TIME_SECONDS_DEFAULT     60
#define HMD_ACCELEROMETER_BLANKING_TIME_SECONDS_MAX         17280

/*** MAIN local structures ***/

/*******************************************************************/
typedef enum {
    HMD_STATE_STARTUP,
    HMD_STATE_MONITORING,
    HMD_STATE_BUTTON,
    HMD_STATE_AIR_QUALITY,
    HMD_STATE_ACCELEROMETER,
    HMD_STATE_ERROR_STACK,
    HMD_STATE_TASK_CHECK,
    HMD_STATE_SLEEP,
    HMD_STATE_LAST
} HMD_state_t;

/*******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned unused :1;
        unsigned daily_downlink :1;
        unsigned lse_status :1;
        unsigned lsi_status :1;
        unsigned accelerometer_enable :1;
        unsigned air_quality_enable :1;
        unsigned temperature_humidity_ens21x_enable :1;
        unsigned temperature_humidity_sht3x_enable :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_status_t;

/*******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned radio_enabled : 1;
        unsigned reset_request :1;
        unsigned downlink_request :1;
        unsigned air_quality_request :1;
        unsigned button_request :1;
        unsigned error_stack_enable :1;
        unsigned monitoring_request :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} HMD_flags_t;

/*******************************************************************/
typedef struct {
    uint32_t monitoring_period_minutes;
    uint32_t air_quality_period_minutes;
    uint32_t accelerometer_blanking_time_seconds;
} HMD_timings_t;

/*******************************************************************/
typedef struct {
    LED_color_t temperature_humidity_reading;
    LED_color_t air_quality_reading;
    LED_color_t accelerometer_reading;
    LED_color_t sigfox_uplink;
    LED_color_t sigfox_downlink;
} HMD_led_color_t;

/*******************************************************************/
typedef struct {
    int32_t threshold_mv;
    LED_color_t led_color;
} HMD_vbatt_indicator_t;

/*******************************************************************/
typedef struct {
    // State machine.
    HMD_state_t state;
    HMD_status_t status;
    volatile HMD_flags_t flags;
    // Monitoring.
    uint32_t monitoring_last_time_seconds;
    uint16_t vbatt_mv;
    uint16_t tamb_tenth_degrees;
    uint8_t hamb_percent;
    // Downlink.
    uint32_t downlink_last_time_seconds;
    HMD_timings_t timings;
    HMD_led_color_t led_color;
    // Error stack.
    uint32_t error_stack_last_time_seconds;
#ifdef HMD_AIR_QUALITY_ENABLE
    uint32_t air_quality_last_time_seconds;
    uint32_t air_quality_acquisition_time_ms;
    ENS16X_device_status_t air_quality_acquisition_status;
    ENS16X_air_quality_data_t air_quality_data;
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    uint8_t accelerometer_state;
    uint32_t accelerometer_last_time_seconds;
#endif
} HMD_context_t;

/*** MAIN local global variables ***/

#ifndef HMD_MODE_CLI
static HMD_context_t hmd_ctx;
#endif

#if (!(defined HMD_MODE_CLI) && (defined HMD_BUTTON_ENABLE))
static const HMD_vbatt_indicator_t HMD_VBATT_INDICATOR[HMD_VBATT_INDICATOR_RANGE] = {
    { 4100, LED_COLOR_GREEN },
    { 4000, LED_COLOR_CYAN },
    { 3900, LED_COLOR_WHITE },
    { 3800, LED_COLOR_YELLOW },
    { 3700, LED_COLOR_BLUE },
    { 3600, LED_COLOR_MAGENTA },
    { 0, LED_COLOR_RED }
};
#endif

/*** MAIN local functions ***/

#if ((defined HMD_BUTTON_ENABLE) && !(defined HMD_MODE_CLI))
/*******************************************************************/
static void _HMD_button_irq_callback(void) {
    hmd_ctx.flags.button_request = 1;
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_load_timings(void) {
    // Local variables.
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t nvm_byte = 0;
    uint16_t generic_u16 = 0;
    // Read monitoring period.
    nvm_status = NVM_read_byte((NVM_ADDRESS_MONITORING_PERIOD_MINUTES + 0), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    generic_u16 = (nvm_byte << 8);
    nvm_status = NVM_read_byte((NVM_ADDRESS_MONITORING_PERIOD_MINUTES + 1), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    generic_u16 |= nvm_byte;
    // Check value.
    if ((generic_u16 < HMD_MONITORING_PERIOD_MINUTES_MIN) || (generic_u16 > HMD_MONITORING_PERIOD_MINUTES_MAX)) {
        // Reset to default value.
        generic_u16 = HMD_MONITORING_PERIOD_MINUTES_DEFAULT;
    }
    hmd_ctx.timings.monitoring_period_minutes = generic_u16;
#ifdef HMD_AIR_QUALITY_ENABLE
    // Read air quality period.
    nvm_status = NVM_read_byte((NVM_ADDRESS_AIR_QUALITY_PERIOD_MINUTES + 0), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    generic_u16 = (nvm_byte << 8);
    nvm_status = NVM_read_byte((NVM_ADDRESS_AIR_QUALITY_PERIOD_MINUTES + 1), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    generic_u16 |= nvm_byte;
    // Check value.
    if ((generic_u16 < HMD_AIR_QUALITY_PERIOD_MINUTES_MIN) || (generic_u16 > HMD_AIR_QUALITY_PERIOD_MINUTES_MAX)) {
        // Reset to default value.
        generic_u16 = HMD_AIR_QUALITY_PERIOD_MINUTES_DEFAULT;
    }
    hmd_ctx.timings.air_quality_period_minutes = generic_u16;
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    // Read accelerometer blanking time.
    nvm_status = NVM_read_byte((NVM_ADDRESS_ACCELEROMETER_BLANKING_TIME_SECONDS + 0), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    generic_u16 = (nvm_byte << 8);
    nvm_status = NVM_read_byte((NVM_ADDRESS_ACCELEROMETER_BLANKING_TIME_SECONDS + 1), &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    generic_u16 |= nvm_byte;
    // Check value.
    if (generic_u16 > HMD_ACCELEROMETER_BLANKING_TIME_SECONDS_MAX) {
       // Reset to default value.
       generic_u16 = HMD_ACCELEROMETER_BLANKING_TIME_SECONDS_DEFAULT;
    }
    hmd_ctx.timings.accelerometer_blanking_time_seconds = generic_u16;
#endif
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_store_timings(HMD_timings_t* timings) {
    // Local variables.
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint16_t generic_u16 = 0;
    // Monitoring period.
    generic_u16 = (timings->monitoring_period_minutes);
    if ((generic_u16 >= HMD_MONITORING_PERIOD_MINUTES_MIN) || (generic_u16 <= HMD_MONITORING_PERIOD_MINUTES_MAX)) {
        // Update context.
        hmd_ctx.timings.monitoring_period_minutes = generic_u16;
        // Write new value in NVM.
        nvm_status = NVM_write_byte((NVM_ADDRESS_MONITORING_PERIOD_MINUTES + 0), (uint8_t) (generic_u16 >> 8));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte((NVM_ADDRESS_MONITORING_PERIOD_MINUTES + 1), (uint8_t) (generic_u16 >> 0));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_MONITORING_PERIOD);
    }
#ifdef HMD_AIR_QUALITY_ENABLE
    // Air quality period.
    generic_u16 = (timings->air_quality_period_minutes);
    if ((generic_u16 >= HMD_AIR_QUALITY_PERIOD_MINUTES_MIN) || (generic_u16 <= HMD_AIR_QUALITY_PERIOD_MINUTES_MAX)) {
        // Update context.
        hmd_ctx.timings.air_quality_period_minutes = generic_u16;
        // Write new value in NVM.
        nvm_status = NVM_write_byte((NVM_ADDRESS_AIR_QUALITY_PERIOD_MINUTES + 0), (uint8_t) (generic_u16 >> 8));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte((NVM_ADDRESS_AIR_QUALITY_PERIOD_MINUTES + 1), (uint8_t) (generic_u16 >> 0));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_AIR_QUALITY_PERIOD);
    }
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    // Accelerometer blanking time.
    generic_u16 = (timings->accelerometer_blanking_time_seconds);
    if (generic_u16 <= HMD_ACCELEROMETER_BLANKING_TIME_SECONDS_MAX) {
        // Update context.
        hmd_ctx.timings.accelerometer_blanking_time_seconds = generic_u16;
        // Write new value in NVM.
        nvm_status = NVM_write_byte((NVM_ADDRESS_ACCELEROMETER_BLANKING_TIME_SECONDS + 0), (uint8_t) (generic_u16 >> 8));
        NVM_stack_error(ERROR_BASE_NVM);
        nvm_status = NVM_write_byte((NVM_ADDRESS_ACCELEROMETER_BLANKING_TIME_SECONDS + 1), (uint8_t) (generic_u16 >> 0));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_ACCELEROMETER_BLANKING_TIME);
    }
#endif
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_load_led_color(void) {
    // Local variables.
    NVM_status_t nvm_status = NVM_SUCCESS;
    uint8_t nvm_byte = 0;
    // Read Sigfox uplink color.
    nvm_status = NVM_read_byte(NVM_ADDRESS_LED_COLOR_SIGFOX_UPLINK, &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    // Check value.
    if (nvm_byte >= LED_COLOR_LAST) {
        // Reset to default value.
        nvm_byte = LED_COLOR_BLUE;
    }
    hmd_ctx.led_color.sigfox_uplink = ((LED_color_t) nvm_byte);
    // Read Sigfox downlink color.
    nvm_status = NVM_read_byte(NVM_ADDRESS_LED_COLOR_SIGFOX_DOWNLINK, &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    // Check value.
    if (nvm_byte >= LED_COLOR_LAST) {
       // Reset to default value.
       nvm_byte = LED_COLOR_CYAN;
    }
    hmd_ctx.led_color.sigfox_downlink = ((LED_color_t) nvm_byte);
    // Read temperature and humidity reading color.
    nvm_status = NVM_read_byte(NVM_ADDRESS_LED_COLOR_TEMPERATURE_HUMIDITY_READING, &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    // Check value.
    if (nvm_byte >= LED_COLOR_LAST) {
        // Reset to default value.
        nvm_byte = LED_COLOR_GREEN;
    }
    hmd_ctx.led_color.temperature_humidity_reading = ((LED_color_t) nvm_byte);
#ifdef HMD_AIR_QUALITY_ENABLE
    // Read air quality reading color.
    nvm_status = NVM_read_byte(NVM_ADDRESS_LED_COLOR_AIR_QUALITY_READING, &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    // Check value.
    if (nvm_byte >= LED_COLOR_LAST) {
        // Reset to default value.
        nvm_byte = LED_COLOR_YELLOW;
    }
    hmd_ctx.led_color.air_quality_reading = ((LED_color_t) nvm_byte);
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    // Read accelerometer reading color.
    nvm_status = NVM_read_byte(NVM_ADDRESS_LED_COLOR_ACCELEROMETER_READING, &nvm_byte);
    NVM_stack_error(ERROR_BASE_NVM);
    // Check value.
    if (nvm_byte >= LED_COLOR_LAST) {
        // Reset to default value.
        nvm_byte = LED_COLOR_MAGENTA;
    }
    hmd_ctx.led_color.accelerometer_reading = ((LED_color_t) nvm_byte);
#endif
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_store_led_color(HMD_led_color_t* led_color) {
    // Local variables.
    NVM_status_t nvm_status = NVM_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    LED_color_t color = LED_COLOR_OFF;
    // Sigfox uplink.
    color = (led_color->sigfox_uplink);
    if (color < LED_COLOR_LAST) {
        // Update context.
        hmd_ctx.led_color.sigfox_uplink = color;
        // Update driver.
        led_status = LED_set_activity_color(LED_ACTIVITY_SIGFOX_UPLINK, color);
        LED_stack_error(ERROR_BASE_LED);
        // Write new value in NVM.
        nvm_status = NVM_write_byte(NVM_ADDRESS_LED_COLOR_SIGFOX_UPLINK, ((uint8_t) color));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_SIGFOX_UPLINK_COLOR);
    }
    // Sigfox downlink.
    color = (led_color->sigfox_downlink);
    if (color < LED_COLOR_LAST) {
        // Update context.
        hmd_ctx.led_color.sigfox_downlink = color;
        // Update driver.
        led_status = LED_set_activity_color(LED_ACTIVITY_SIGFOX_DOWNLINK, color);
        LED_stack_error(ERROR_BASE_LED);
        // Write new value in NVM.
        nvm_status = NVM_write_byte(NVM_ADDRESS_LED_COLOR_SIGFOX_DOWNLINK, ((uint8_t) color));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_SIGFOX_DOWNLINK_COLOR);
    }
    // Temperature and humidity reading.
    color = (led_color->temperature_humidity_reading);
    if (color < LED_COLOR_LAST) {
        // Update context.
        hmd_ctx.led_color.temperature_humidity_reading = color;
        // Update driver.
        led_status = LED_set_activity_color(LED_ACTIVITY_TEMPERATURE_HUMIDITY_READING, color);
        LED_stack_error(ERROR_BASE_LED);
        // Write new value in NVM.
        nvm_status = NVM_write_byte(NVM_ADDRESS_LED_COLOR_TEMPERATURE_HUMIDITY_READING, ((uint8_t) color));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_TEMPERATURE_HUMIDITY_READING_COLOR);
    }
#ifdef HMD_AIR_QUALITY_ENABLE
    // Air quality reading.
    color = (led_color->air_quality_reading);
    if (color < LED_COLOR_LAST) {
        // Update context.
        hmd_ctx.led_color.air_quality_reading = color;
        // Update driver.
        led_status = LED_set_activity_color(LED_ACTIVITY_AIR_QUALITY_READING, color);
        LED_stack_error(ERROR_BASE_LED);
        // Write new value in NVM.
        nvm_status = NVM_write_byte(NVM_ADDRESS_LED_COLOR_AIR_QUALITY_READING, ((uint8_t) color));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_AIR_QUALITY_READING_COLOR);
    }
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    // Accelerometer reading
    color = (led_color->accelerometer_reading);
    if (color < LED_COLOR_LAST) {
        // Update context.
        hmd_ctx.led_color.accelerometer_reading = color;
        // Update driver.
        led_status = LED_set_activity_color(LED_ACTIVITY_ACCELEROMETER_READING, color);
        LED_stack_error(ERROR_BASE_LED);
        // Write new value in NVM.
        nvm_status = NVM_write_byte(NVM_ADDRESS_LED_COLOR_ACCELEROMETER_READING, ((uint8_t) color));
        NVM_stack_error(ERROR_BASE_NVM);
    }
    else {
        ERROR_stack_add(ERROR_SIGFOX_EP_DL_ACCELEROMETER_READING_COLOR);
    }
#endif
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_init_context(void) {
    // Init context.
    hmd_ctx.state = HMD_STATE_STARTUP;
    hmd_ctx.status.all = 0;
    hmd_ctx.flags.all = 0;
    hmd_ctx.flags.radio_enabled = 1;
    hmd_ctx.flags.downlink_request = 1;
    hmd_ctx.flags.error_stack_enable = 1;
    hmd_ctx.monitoring_last_time_seconds = 0;
    hmd_ctx.downlink_last_time_seconds = 0;
    hmd_ctx.error_stack_last_time_seconds = 0;
    hmd_ctx.vbatt_mv = SIGFOX_EP_ERROR_VALUE_ANALOG_16BITS;
    hmd_ctx.tamb_tenth_degrees = SIGFOX_EP_ERROR_VALUE_TEMPERATURE;
    hmd_ctx.hamb_percent = SIGFOX_EP_ERROR_VALUE_HUMIDITY;
#ifdef HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE
    hmd_ctx.status.temperature_humidity_sht3x_enable = 1;
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE
    hmd_ctx.status.temperature_humidity_ens21x_enable = 1;
#endif
#ifdef HMD_AIR_QUALITY_ENABLE
    hmd_ctx.status.air_quality_enable = 1;
    hmd_ctx.air_quality_last_time_seconds = 0;
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    hmd_ctx.status.accelerometer_enable = 1;
    hmd_ctx.accelerometer_state = 0;
    hmd_ctx.accelerometer_last_time_seconds = 0;
#endif
    // Load configuration from NVM.
    _HMD_load_timings();
    _HMD_load_led_color();
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
#if ((defined HMD_BUTTON_ENABLE) && !(defined HMD_MODE_CLI))
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
#if ((defined HMD_BUTTON_ENABLE) && !(defined HMD_MODE_CLI))
    button_status = BUTTON_init(&_HMD_button_irq_callback);
    BUTTON_stack_error(ERROR_BASE_BUTTON);
    button_status = BUTTON_enable_interrupt();
    BUTTON_stack_error(ERROR_BASE_BUTTON);
#endif
    led_status = LED_init();
    LED_stack_error(ERROR_BASE_LED);
#ifndef HMD_MODE_CLI
    // Init device configuration.
    _HMD_store_timings(&hmd_ctx.timings);
    _HMD_store_led_color(&hmd_ctx.led_color);
#endif
}

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_update_battery_voltage(void) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    int32_t vbatt = 0;
    // Reset data.
    hmd_ctx.vbatt_mv = SIGFOX_EP_ERROR_VALUE_ANALOG_16BITS;
    // Perform battery voltage measurement.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VBATT_MV, &vbatt);
    ANALOG_stack_error(ERROR_BASE_ANALOG);
    if (analog_status == ANALOG_SUCCESS) {
        // Voltage hysteresis for radio.
        if (vbatt < HMD_RADIO_OFF_VSTR_THRESHOLD_MV) {
            hmd_ctx.flags.radio_enabled = 0;
        }
        if (vbatt > HMD_RADIO_ON_VSTR_THRESHOLD_MV) {
            hmd_ctx.flags.radio_enabled = 1;
        }
        // Update data.
        hmd_ctx.vbatt_mv = (uint16_t) vbatt;
    }
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_update_temperature_humidity(void) {
    // Local variables.
#if ((defined HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE) || (defined HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE))
    MATH_status_t math_status = MATH_SUCCESS;
    int32_t temperature = 0;
    uint32_t temperature_signed_magnitude;
    int32_t humidity = 0;
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE
    ENS21X_status_t ens21x_status = ENS21X_SUCCESS;
#endif
    // Reset data.
    hmd_ctx.tamb_tenth_degrees = SIGFOX_EP_ERROR_VALUE_TEMPERATURE;
    hmd_ctx.hamb_percent = SIGFOX_EP_ERROR_VALUE_HUMIDITY;
#if ((defined HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE) || (defined HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE))
    // Turn LED on.
    LED_set_activity(LED_ACTIVITY_TEMPERATURE_HUMIDITY_READING);
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE
    // Get temperature and humidity from ENS210.
    ens21x_status = ENS21X_get_temperature_humidity(I2C_ADDRESS_ENS210, &temperature, &humidity);
    ENS21X_stack_error(ERROR_BASE_ENS210);
    // Check status.
    if (ens21x_status == ENS21X_SUCCESS) {
       // Convert temperature.
       math_status = MATH_integer_to_signed_magnitude(temperature, 11, &temperature_signed_magnitude);
       MATH_stack_error(ERROR_BASE_MATH);
       if (math_status == MATH_SUCCESS) {
           hmd_ctx.tamb_tenth_degrees = (uint16_t) temperature_signed_magnitude;
       }
       hmd_ctx.hamb_percent = (uint8_t) humidity;
    }
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE
    if ((hmd_ctx.tamb_tenth_degrees == SIGFOX_EP_ERROR_VALUE_TEMPERATURE) || (hmd_ctx.hamb_percent == SIGFOX_EP_ERROR_VALUE_HUMIDITY)) {
        // Get temperature and humidity from SHT30.
        sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30, &temperature, &humidity);
        SHT3X_stack_error(ERROR_BASE_SHT30);
        // Check status.
        if (sht3x_status == SHT3X_SUCCESS) {
           // Convert temperature.
           math_status = MATH_integer_to_signed_magnitude(temperature, 11, &temperature_signed_magnitude);
           MATH_stack_error(ERROR_BASE_MATH);
           if (math_status == MATH_SUCCESS) {
               hmd_ctx.tamb_tenth_degrees = (uint16_t) temperature_signed_magnitude;
           }
           hmd_ctx.hamb_percent = (uint8_t) humidity;
        }
    }
#endif
#if ((defined HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE) || (defined HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE))
    // Turn LED off.
    LED_set_activity(LED_ACTIVITY_NONE);
#endif
}
#endif

#if (!(defined HMD_MODE_CLI) && (defined HMD_AIR_QUALITY_ENABLE))
/*******************************************************************/
static void _HMD_update_air_quality(void) {
    // Local variables.
    ENS16X_status_t ens16x_status = ENS16X_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    ENS16X_air_quality_data_t air_quality_data;
    int32_t tamb_tenth_degrees = ((hmd_ctx.tamb_tenth_degrees != SIGFOX_EP_ERROR_VALUE_TEMPERATURE) ? hmd_ctx.tamb_tenth_degrees : 25);;
    int32_t hamb_percent = ((hmd_ctx.hamb_percent != SIGFOX_EP_ERROR_VALUE_HUMIDITY) ? hmd_ctx.hamb_percent : 50);
    // Reset data.
    hmd_ctx.air_quality_acquisition_time_ms = 0;
    hmd_ctx.air_quality_acquisition_status.validity_flag = ENS16X_VALIDITY_FLAG_INVALID_OUTPUT;
    hmd_ctx.air_quality_data.tvoc_ppb = SIGFOX_EP_ERROR_VALUE_TVOC;
    hmd_ctx.air_quality_data.eco2_ppm = SIGFOX_EP_ERROR_VALUE_ECO2;
    hmd_ctx.air_quality_data.aqi_uba = SIGFOX_EP_ERROR_VALUE_AQI_UBA;
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    hmd_ctx.air_quality_data.aqi_s = SIGFOX_EP_ERROR_VALUE_AQI_S;
#endif
    // Read device status.
    ens16x_status = ENS16X_get_device_status(I2C_ADDRESS_ENS16X, &hmd_ctx.air_quality_acquisition_status);
    ENS16X_stack_error(ERROR_BASE_ENS16X);
    // Check if sensor is running.
    if (hmd_ctx.air_quality_acquisition_status.statas == 0) {
        // Start acquisition.
        ens16x_status = ENS16X_set_operating_mode(I2C_ADDRESS_ENS16X, ENS16X_OPERATING_MODE_IDLE);
        ENS16X_stack_error(ERROR_BASE_ENS16X);
        ens16x_status = ENS16X_set_operating_mode(I2C_ADDRESS_ENS16X, HMD_AIR_QUALITY_ACQUISITION_MODE);
        ENS16X_stack_error(ERROR_BASE_ENS16X);
    }
    // Set RHT compensation.
    ens16x_status = ENS16X_set_temperature_humidity(I2C_ADDRESS_ENS16X, tamb_tenth_degrees, hamb_percent);
    ENS16X_stack_error(ERROR_BASE_ENS16X);
    // Directly exit in case of POR.
    if (hmd_ctx.state == HMD_STATE_STARTUP) goto errors;
    // Directly exit in case of error.
    if (ens16x_status == ENS16X_SUCCESS) {
        // Wait for acquisition time.
        do {
            // Reload watchdog.
            IWDG_reload();
            LED_set_activity(LED_ACTIVITY_NONE);
            // Low power acquisition delay.
            lptim_status = LPTIM_delay_milliseconds((HMD_AIR_QUALITY_ACQUISITION_DELAY_MS - HMD_AIR_QUALITY_ACQUISITION_LED_BLINK_MS), LPTIM_DELAY_MODE_STOP);
            LPTIM_stack_error(ERROR_BASE_LPTIM);
            // Blink yellow LED.
            LED_set_activity(LED_ACTIVITY_AIR_QUALITY_READING);
            // LED delay.
            lptim_status = LPTIM_delay_milliseconds(HMD_AIR_QUALITY_ACQUISITION_LED_BLINK_MS, LPTIM_DELAY_MODE_STOP);
            LPTIM_stack_error(ERROR_BASE_LPTIM);
            // Update duration.
            hmd_ctx.air_quality_acquisition_time_ms += HMD_AIR_QUALITY_ACQUISITION_DELAY_MS;
            // Read device status.
            ens16x_status = ENS16X_get_device_status(I2C_ADDRESS_ENS16X, &hmd_ctx.air_quality_acquisition_status);
            ENS16X_stack_error(ERROR_BASE_ENS16X);
            // Check status.
            if (ens16x_status != ENS16X_SUCCESS) break;
            // Check data validity.
            if (hmd_ctx.air_quality_acquisition_status.newdat != 0) {
                // Read data.
                ens16x_status = ENS16X_read_air_quality(I2C_ADDRESS_ENS16X, &air_quality_data);
                ENS16X_stack_error(ERROR_BASE_ENS16X);
                // Check status.
                if (ens16x_status != ENS16X_SUCCESS) break;
                // Check validity flag and minimum time.
                if ((hmd_ctx.air_quality_acquisition_status.validity_flag == ENS16X_VALIDITY_FLAG_NORMAL_OPERATION) && (hmd_ctx.air_quality_acquisition_time_ms >= HMD_AIR_QUALITY_ACQUISITION_TIME_MIN_MS)) {
                    // Update data.
                    hmd_ctx.air_quality_data.tvoc_ppb = (air_quality_data.tvoc_ppb);
                    hmd_ctx.air_quality_data.eco2_ppm = (air_quality_data.eco2_ppm);
                    hmd_ctx.air_quality_data.aqi_uba = (air_quality_data.aqi_uba);
#ifdef ENS16X_DRIVER_DEVICE_ENS161
                    hmd_ctx.air_quality_data.aqi_s = (air_quality_data.aqi_s);
#endif
                    break;
                }
            }
        }
        while (hmd_ctx.air_quality_acquisition_time_ms < HMD_AIR_QUALITY_ACQUISITION_TIME_MAX_MS);
    }
    // Turn LED off.
    LED_set_activity(LED_ACTIVITY_NONE);
errors:
    return;
}
#endif

#ifndef HMD_MODE_CLI
/*******************************************************************/
static void _HMD_send_sigfox_message(SIGFOX_EP_API_application_message_t* sigfox_ep_application_message) {
    // Local variables.
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    SIGFOX_EP_API_message_status_t message_status;
    SIGFOX_EP_dl_payload_t dl_payload;
    HMD_timings_t timings;
    HMD_led_color_t led_color;
    int16_t dl_rssi = 0;
    uint8_t status = 0;
    // Directly exit of the radio is disabled due to low battery voltage.
    if (hmd_ctx.flags.radio_enabled == 0) goto errors;
    // Check downlink request.
    (sigfox_ep_application_message->bidirectional_flag) = ((hmd_ctx.flags.downlink_request != 0) ? SIGFOX_TRUE : SIGFOX_FALSE);
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_check_status(0);
    // Send message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(sigfox_ep_application_message);
    SIGFOX_EP_API_check_status(0);
    // Check bidirectional flag.
    if (hmd_ctx.flags.downlink_request != 0) {
        // Clear request and reset status.
        hmd_ctx.flags.downlink_request = 0;
        hmd_ctx.status.daily_downlink = 0;
        // Read message status.
        message_status = SIGFOX_EP_API_get_message_status();
        // Check if downlink data is available.
        if (message_status.field.dl_frame != 0) {
            // Update status.
            hmd_ctx.status.daily_downlink = 1;
            // Read downlink payload.
            sigfox_ep_api_status = SIGFOX_EP_API_get_dl_payload(dl_payload.frame, SIGFOX_DL_PAYLOAD_SIZE_BYTES, &dl_rssi);
            SIGFOX_EP_API_check_status(0);
            if (sigfox_ep_api_status == SIGFOX_EP_API_SUCCESS) {
                // Parse payload.
                switch (dl_payload.op_code) {
                case SIGFOX_EP_DL_OP_CODE_NOP:
                    // Nothing to do.
                    break;
                case SIGFOX_EP_DL_OP_CODE_RESET:
                    // Set reset request.
                    hmd_ctx.flags.reset_request = 1;
                    break;
                case SIGFOX_EP_DL_OP_CODE_SET_TIMINGS:
                    // Build timing structure.
                    timings.monitoring_period_minutes = dl_payload.set_timings.monitoring_period_minutes;
                    timings.air_quality_period_minutes = dl_payload.set_timings.air_quality_period_minutes;
                    timings.accelerometer_blanking_time_seconds = dl_payload.set_timings.accelerometer_blanking_time_seconds;
                    // Check and store new configuration.
                    _HMD_store_timings(&timings);
                    break;
                case SIGFOX_EP_DL_OP_CODE_SET_LED_COLOR:
                    // Build LED configuration structure.
                    led_color.sigfox_uplink = dl_payload.set_led_color.sigfox_uplink;
                    led_color.sigfox_downlink = dl_payload.set_led_color.sigfox_downlink;
                    led_color.temperature_humidity_reading = dl_payload.set_led_color.temperature_humidity_reading;
                    led_color.air_quality_reading = dl_payload.set_led_color.air_quality_reading;
                    led_color.accelerometer_reading = dl_payload.set_led_color.accelerometer_reading;
                    // Check and store new configuration.
                    _HMD_store_led_color(&led_color);
                    break;
                default:
                    ERROR_stack_add(ERROR_SIGFOX_EP_DL_OP_CODE);
                    break;
                }
            }
        }
    }
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
    SIGFOX_EP_API_application_message_t sigfox_ep_application_message;
    SIGFOX_EP_ul_payload_startup_t sigfox_ep_ul_payload_startup;
    SIGFOX_EP_ul_payload_monitoring_t sigfox_ep_ul_payload_monitoring;
    ERROR_code_t error_code = 0;
    uint8_t sigfox_ep_ul_payload_error_stack[SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK];
#ifdef HMD_BUTTON_ENABLE
    LED_status_t led_status = LED_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    LED_color_t led_color = LED_COLOR_OFF;
#endif
#ifdef HMD_AIR_QUALITY_ENABLE
    SIGFOX_EP_ul_payload_air_quality_t sigfox_ep_ul_payload_air_quality;
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    FXLS89XXXX_status_t fxls89xxxx_status = FXLS89XXXX_SUCCESS;
    FXLS89XXXX_int_src1_t fxls89xxxx_int_src1;
    FXLS89XXXX_int_src2_t fxls89xxxx_int_src2;
    SIGFOX_EP_ul_payload_accelerometer_t sigfox_ul_payload_accelerometer;
#endif
    uint8_t generic_u8 = 0;
    uint32_t generic_u32 = 0;
    // Application message default parameters.
    sigfox_ep_application_message.common_parameters.number_of_frames = 3;
    sigfox_ep_application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    sigfox_ep_application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
    sigfox_ep_application_message.bidirectional_flag = SIGFOX_FALSE;
    sigfox_ep_application_message.ul_payload = SIGFOX_NULL;
    sigfox_ep_application_message.ul_payload_size_bytes = 0;
    // Main loop.
    while (1) {
        // Perform state machine.
        switch (hmd_ctx.state) {
        case HMD_STATE_STARTUP:
            IWDG_reload();
#ifdef HMD_AIR_QUALITY_ENABLE
            // Start air quality sensor
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
            _HMD_update_temperature_humidity();
            _HMD_update_air_quality();
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
#endif
            // Fill reset reason and software version.
            sigfox_ep_ul_payload_startup.reset_reason = PWR_get_reset_flags();
            sigfox_ep_ul_payload_startup.major_version = GIT_MAJOR_VERSION;
            sigfox_ep_ul_payload_startup.minor_version = GIT_MINOR_VERSION;
            sigfox_ep_ul_payload_startup.commit_index = GIT_COMMIT_INDEX;
            sigfox_ep_ul_payload_startup.commit_id = GIT_COMMIT_ID;
            sigfox_ep_ul_payload_startup.dirty_flag = GIT_DIRTY_FLAG;
            // Clear reset flags.
            PWR_clear_reset_flags();
            // Send startup message.
            sigfox_ep_application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
            sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_startup.frame);
            sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP;
            _HMD_send_sigfox_message(&sigfox_ep_application_message);
            // Compute next state.
            hmd_ctx.state = HMD_STATE_ERROR_STACK;
            break;
        case HMD_STATE_MONITORING:
            IWDG_reload();
            // Turn sensors on.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_ACTIVE);
            // Measure related data.
            _HMD_update_battery_voltage();
            _HMD_update_temperature_humidity();
            // Turn sensors off.
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG);
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
            // Update status.
            rcc_status = RCC_get_status(RCC_CLOCK_LSI, &generic_u8);
            RCC_stack_error(ERROR_BASE_RCC);
            hmd_ctx.status.lsi_status = (generic_u8 == 0) ? 0b0 : 0b1;
            rcc_status = RCC_get_status(RCC_CLOCK_LSE, &generic_u8);
            RCC_stack_error(ERROR_BASE_RCC);
            hmd_ctx.status.lse_status = (generic_u8 == 0) ? 0b0 : 0b1;
            // Build frame.
            sigfox_ep_ul_payload_monitoring.status = hmd_ctx.status.all;
            sigfox_ep_ul_payload_monitoring.vbatt_mv = hmd_ctx.vbatt_mv;
            sigfox_ep_ul_payload_monitoring.unused = 0;
            sigfox_ep_ul_payload_monitoring.tamb_tenth_degrees = hmd_ctx.tamb_tenth_degrees;
            sigfox_ep_ul_payload_monitoring.hamb_percent = hmd_ctx.hamb_percent;
            // Send uplink monitoring frame.
            sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_monitoring.frame);
            sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING;
            _HMD_send_sigfox_message(&sigfox_ep_application_message);
            // Clear request.
            hmd_ctx.flags.monitoring_request = 0;
            // Compute next state.
            hmd_ctx.state = HMD_STATE_ERROR_STACK;
            break;
#ifdef HMD_BUTTON_ENABLE
        case HMD_STATE_BUTTON:
            IWDG_reload();
            // Turn sensors on.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
            // Measure related data.
            _HMD_update_battery_voltage();
            // Turn sensors off.
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG);
            // Compute LED color.
            if (hmd_ctx.vbatt_mv == SIGFOX_EP_ERROR_VALUE_ANALOG_16BITS) {
                led_color = LED_COLOR_OFF;
            }
            else {
                for (generic_u8 = 0; generic_u8 < HMD_VBATT_INDICATOR_RANGE; generic_u8++) {
                    if (hmd_ctx.vbatt_mv >= HMD_VBATT_INDICATOR[generic_u8].threshold_mv) {
                        // Turn LED on.
                        led_color = HMD_VBATT_INDICATOR[generic_u8].led_color;
                        break;
                    }
                }
            }
            // Turn LED on.
            led_status = LED_set_color(led_color);
            LED_stack_error(ERROR_BASE_LED);
            // Delay.
            lptim_status = LPTIM_delay_milliseconds(HMD_VBATT_INDICATOR_DELAY_MS, LPTIM_DELAY_MODE_STOP);
            LPTIM_stack_error(ERROR_BASE_LPTIM);
            // Turn LED off.
            led_status = LED_set_color(LED_COLOR_OFF);
            LED_stack_error(ERROR_BASE_LED);
            // Clear request.
            hmd_ctx.flags.button_request = 0;
            // Compute next state.
            hmd_ctx.state = HMD_STATE_TASK_CHECK;
            break;
#endif
#ifdef HMD_AIR_QUALITY_ENABLE
        case HMD_STATE_AIR_QUALITY:
            IWDG_reload();
            // Turn sensors on.
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
            // Measure related data.
            _HMD_update_temperature_humidity();
            _HMD_update_air_quality();
            // Turn sensors off.
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
            // Build frame.
            sigfox_ep_ul_payload_air_quality.acquisition_duration_tens_seconds = (hmd_ctx.air_quality_acquisition_time_ms / 10000);
            sigfox_ep_ul_payload_air_quality.acquisition_status = hmd_ctx.air_quality_acquisition_status.validity_flag;
            sigfox_ep_ul_payload_air_quality.tvoc_ppb = hmd_ctx.air_quality_data.tvoc_ppb;
            sigfox_ep_ul_payload_air_quality.eco2_ppm = hmd_ctx.air_quality_data.eco2_ppm;
            sigfox_ep_ul_payload_air_quality.aqi_uba = hmd_ctx.air_quality_data.aqi_uba;
#ifdef ENS16X_DRIVER_DEVICE_ENS161
            sigfox_ep_ul_payload_air_quality.aqi_s = hmd_ctx.air_quality_data.aqi_s;
#else
            sigfox_ep_ul_payload_air_quality.aqi_s = SIGFOX_EP_ERROR_VALUE_AQI_S;
#endif
            // Send uplink air quality frame.
            sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_air_quality.frame);
            sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_AIR_QUALITY;
            _HMD_send_sigfox_message(&sigfox_ep_application_message);
            // Clear request.
            hmd_ctx.flags.air_quality_request = 0;
            // Compute next state.
            hmd_ctx.state = HMD_STATE_ERROR_STACK;
            break;
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
        case HMD_STATE_ACCELEROMETER:
            IWDG_reload();
            // Update accelerometer state and last event time.
            hmd_ctx.accelerometer_state = 0;
            hmd_ctx.accelerometer_last_time_seconds = RTC_get_uptime_seconds();
            // Disable interrupt on MCU side.
            SENSORS_HW_disable_sensor_interrupt();
            // Turn sensors on.
            LED_set_activity(LED_ACTIVITY_ACCELEROMETER_READING);
            POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
            // Disable accelerometer.
            fxls89xxxx_status = FXLS89XXXX_write_configuration(I2C_ADDRESS_FXLS8974CF, FXLS89XXXX_SLEEP_CONFIGURATION, FXLS89XXXX_SLEEP_CONFIGURATION_SIZE);
            FXLS89XXXX_stack_error(ERROR_BASE_FXLS8974CF);
            // Read interrupt status.
            fxls89xxxx_status = FXLS89XXXX_clear_interrupt(I2C_ADDRESS_FXLS8974CF, &fxls89xxxx_int_src1, &fxls89xxxx_int_src2);
            FXLS89XXXX_stack_error(ERROR_BASE_FXLS8974CF);
            // Turn sensors off.
            POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
            LED_set_activity(LED_ACTIVITY_NONE);
            // Check status.
            sigfox_ul_payload_accelerometer.event_source = ((fxls89xxxx_status == FXLS89XXXX_SUCCESS) ? (fxls89xxxx_int_src1.all) : 0);
            // Send uplink air quality frame.
            sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ul_payload_accelerometer.frame);
            sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_ACCELEROMETER;
            _HMD_send_sigfox_message(&sigfox_ep_application_message);
            // Compute next state.
            hmd_ctx.state = HMD_STATE_TASK_CHECK;
            break;
#endif
        case HMD_STATE_ERROR_STACK:
            IWDG_reload();
            // Check if blanking time elapsed.
            if (hmd_ctx.flags.error_stack_enable != 0) {
                // Import Sigfox library error stack.
                ERROR_import_sigfox_stack();
                // Check stack.
                if (ERROR_stack_is_empty() == 0) {
                    // Read error stack.
                    for (generic_u8 = 0; generic_u8 < (SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK >> 1); generic_u8++) {
                        error_code = ERROR_stack_read();
                        sigfox_ep_ul_payload_error_stack[(generic_u8 << 1) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
                        sigfox_ep_ul_payload_error_stack[(generic_u8 << 1) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
                    }
                    // Send error stack frame.
                    sigfox_ep_application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
                    sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_error_stack);
                    sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK;
                    _HMD_send_sigfox_message(&sigfox_ep_application_message);
                    // Disable error stack sending.
                    hmd_ctx.flags.error_stack_enable = 0;
                    hmd_ctx.error_stack_last_time_seconds = RTC_get_uptime_seconds();
                }
            }
            // Compute next state.
            hmd_ctx.state = HMD_STATE_TASK_CHECK;
            break;
        case HMD_STATE_TASK_CHECK:
            IWDG_reload();
            // Read uptime.
            generic_u32 = RTC_get_uptime_seconds();
            // Periodic monitoring.
            if (generic_u32 >= (hmd_ctx.monitoring_last_time_seconds + (hmd_ctx.timings.monitoring_period_minutes * 60))) {
               // Set request and update last time.
               hmd_ctx.flags.monitoring_request = 1;
               hmd_ctx.monitoring_last_time_seconds = generic_u32;
            }
            // Periodic downlink.
            if (generic_u32 >= (hmd_ctx.downlink_last_time_seconds + HMD_DOWNLINK_PERIOD_SECONDS)) {
               // Set request and update last time.
               hmd_ctx.flags.downlink_request = 1;
               hmd_ctx.downlink_last_time_seconds = generic_u32;
            }
            // Error stack.
            if (generic_u32 >= (hmd_ctx.error_stack_last_time_seconds + HMD_ERROR_STACK_BLANKING_TIME_SECONDS)) {
               // Enable error stack message.
               hmd_ctx.flags.error_stack_enable = 1;
            }
#ifdef HMD_AIR_QUALITY_ENABLE
            if (generic_u32 >= (hmd_ctx.air_quality_last_time_seconds + (hmd_ctx.timings.air_quality_period_minutes * 60))) {
               // Set request and update last time.
               hmd_ctx.flags.air_quality_request = 1;
               hmd_ctx.air_quality_last_time_seconds = generic_u32;
            }
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
            // Check accelerometer blanking time.
            if ((generic_u32 >= (hmd_ctx.accelerometer_last_time_seconds + hmd_ctx.timings.accelerometer_blanking_time_seconds)) && (hmd_ctx.accelerometer_state == 0)) {
                // Turn sensors on.
                LED_set_activity(LED_ACTIVITY_ACCELEROMETER_READING);
                POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_SLEEP);
                // Enable accelerometer.
                fxls89xxxx_status = FXLS89XXXX_write_configuration(I2C_ADDRESS_FXLS8974CF, FXLS89XXXX_ACTIVE_CONFIGURATION, FXLS89XXXX_ACTIVE_CONFIGURATION_SIZE);
                FXLS89XXXX_stack_error(ERROR_BASE_FXLS8974CF);
                // Turn sensors off.
                POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
                LED_set_activity(LED_ACTIVITY_NONE);
                // Enable interrupt on MCU side.
                SENSORS_HW_enable_sensor_interrupt();
                // Update state.
                hmd_ctx.accelerometer_state = 1;
            }
#endif
            // Go to sleep by default.
            hmd_ctx.state = HMD_STATE_SLEEP;
            // Check wake-up flags.
#ifdef HMD_AIR_QUALITY_ENABLE
            if (hmd_ctx.flags.air_quality_request != 0) {
                hmd_ctx.state = HMD_STATE_AIR_QUALITY;
            }
#endif
            if (hmd_ctx.flags.monitoring_request != 0) {
                hmd_ctx.state = HMD_STATE_MONITORING;
            }
#ifdef HMD_BUTTON_ENABLE
            if (hmd_ctx.flags.button_request != 0) {
                hmd_ctx.state = HMD_STATE_BUTTON;
            }
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
            if (GPIO_read(&GPIO_SENSOR_IRQ) != 0) {
                hmd_ctx.state = HMD_STATE_ACCELEROMETER;
            }
#endif
            if (hmd_ctx.state != HMD_STATE_SLEEP) {
                // Calibrate clocks.
                rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
                RCC_stack_error(ERROR_BASE_RCC);
            }
            break;
        case HMD_STATE_SLEEP:
            // Check reset request.
            if (hmd_ctx.flags.reset_request != 0) {
                PWR_software_reset();
            }
            // Enter stop mode.
            IWDG_reload();
            PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP);
            IWDG_reload();
            // Check wake-up reason.
            hmd_ctx.state = HMD_STATE_TASK_CHECK;
            break;
        default:
            // Unknown state.
            hmd_ctx.state = HMD_STATE_TASK_CHECK;
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
