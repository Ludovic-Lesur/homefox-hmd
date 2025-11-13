/*
 * power.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "power.h"

#include "analog.h"
#include "ens16x.h"
#include "error.h"
#include "error_base.h"
#include "fxls89xxxx.h"
#include "gpio.h"
#include "hmd_flags.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "rfe.h"
#include "sht3x.h"
#include "sx126x.h"
#include "types.h"

/*** POWER local global variables ***/

static uint32_t power_domain_state[POWER_DOMAIN_LAST] = { [0 ... (POWER_DOMAIN_LAST - 1)] = 0 };

/*** POWER local functions ***/

/*******************************************************************/
#define _POWER_stack_driver_error(driver_status, driver_success, driver_error_base, power_status) { \
    if (driver_status != driver_success) { \
        ERROR_stack_add(driver_error_base + driver_status); \
        ERROR_stack_add(ERROR_BASE_POWER + power_status); \
    } \
}

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
    // Local variables.
    uint8_t idx = 0;
    // Init context.
    for (idx = 0; idx < POWER_DOMAIN_LAST; idx++) {
        power_domain_state[idx] = 0;
    }
    // Init power control pins.
    GPIO_configure(&GPIO_MNTR_EN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/*******************************************************************/
void POWER_enable(POWER_requester_id_t requester_id, POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
#ifdef HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE
    ENS21X_status_t ens21x_status = ENS21X_SUCCESS;
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
#endif
#ifdef HMD_AIR_QUALITY_ENABLE
    ENS16X_status_t ens16x_status = ENS16X_SUCCESS;
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    FXLS89XXXX_status_t fxls89xxxx_status = FXLS89XXXX_SUCCESS;
#endif
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    uint32_t delay_ms = 0;
    uint8_t action_required = 0;
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    action_required = ((power_domain_state[domain] == 0) ? 1 : 0);
    // Update state.
    power_domain_state[domain] |= (0b1 << requester_id);
    // Directly exit if this is not the first request.
    if (action_required == 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Turn analog front-end on.
        GPIO_write(&GPIO_MNTR_EN, 1);
        delay_ms = POWER_ON_DELAY_MS_ANALOG;
        // Init attached drivers.
        analog_status = ANALOG_init();
        _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
        break;
    case POWER_DOMAIN_SENSORS:
        // Turn digital sensors.
        GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_SENSORS;
        // Init attached drivers.
#ifdef HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE
        ens21x_status = ENS21X_init();
        _POWER_stack_driver_error(ens21x_status, ENS21X_SUCCESS, ERROR_BASE_ENS210, POWER_ERROR_DRIVER_ENS21X);
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE
        sht3x_status = SHT3X_init();
        _POWER_stack_driver_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30, POWER_ERROR_DRIVER_SHT3X);
#endif
#ifdef HMD_AIR_QUALITY_ENABLE
        ens16x_status = ENS16X_init();
        _POWER_stack_driver_error(ens16x_status, ENS16X_SUCCESS, ERROR_BASE_ENS16X, POWER_ERROR_DRIVER_ENS16X);
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
        fxls89xxxx_status = FXLS89XXXX_init();
        _POWER_stack_driver_error(fxls89xxxx_status, FXLS89XXXX_SUCCESS, ERROR_BASE_FXLS8974CF, POWER_ERROR_DRIVER_FXLS89XXXX);
#endif
        // Turn green LED on.
        led_status = LED_set_color(LED_COLOR_GREEN);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        break;
    case POWER_DOMAIN_TCXO:
        // Turn TCXO on.
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_TCXO;
        break;
    case POWER_DOMAIN_RADIO:
        // Turn radio on.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_RADIO;
        // Init attached drivers.
        sx126x_status = SX126X_init();
        _POWER_stack_driver_error(sx126x_status, SX126X_SUCCESS, ERROR_BASE_SX1261, POWER_ERROR_DRIVER_SX126X);
        rfe_status = RFE_init();
        _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
        // Turn blue LED on.
        led_status = LED_set_color(LED_COLOR_BLUE);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        break;
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    // Power on delay.
    if (delay_ms != 0) {
        lptim_status = LPTIM_delay_milliseconds(delay_ms, delay_mode);
        _POWER_stack_driver_error(lptim_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM, POWER_ERROR_DRIVER_LPTIM);
    }
errors:
    return;
}

/*******************************************************************/
void POWER_disable(POWER_requester_id_t requester_id, POWER_domain_t domain) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
#ifdef HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE
    ENS21X_status_t ens21x_status = ENS21X_SUCCESS;
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
#endif
#ifdef HMD_AIR_QUALITY_ENABLE
    ENS16X_status_t ens16x_status = ENS16X_SUCCESS;
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    FXLS89XXXX_status_t fxls89xxxx_status = FXLS89XXXX_SUCCESS;
#endif
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    if (power_domain_state[domain] == 0) goto errors;
    // Update state.
    power_domain_state[domain] &= ~(0b1 << requester_id);
    // Directly exit if this is not the last request.
    if (power_domain_state[domain] != 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Release attached drivers.
        analog_status = ANALOG_de_init();
        _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
        // Turn analog front-end off.
        GPIO_write(&GPIO_MNTR_EN, 0);
        break;
    case POWER_DOMAIN_SENSORS:
        // Release attached drivers.
#ifdef HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE
        ens21x_status = ENS21X_de_init();
        _POWER_stack_driver_error(ens21x_status, ENS21X_SUCCESS, ERROR_BASE_ENS210, POWER_ERROR_DRIVER_ENS21X);
#endif
#ifdef HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE
        sht3x_status = SHT3X_de_init();
        _POWER_stack_driver_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30, POWER_ERROR_DRIVER_SHT3X);
#endif
#ifdef HMD_AIR_QUALITY_ENABLE
        ens16x_status = ENS16X_de_init();
        _POWER_stack_driver_error(ens16x_status, ENS16X_SUCCESS, ERROR_BASE_ENS16X, POWER_ERROR_DRIVER_ENS16X);
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
        fxls89xxxx_status = FXLS89XXXX_de_init();
        _POWER_stack_driver_error(fxls89xxxx_status, FXLS89XXXX_SUCCESS, ERROR_BASE_FXLS8974CF, POWER_ERROR_DRIVER_FXLS89XXXX);
#endif
        // Turn digital sensors off.
        GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
        // Turn LED off.
        led_status = LED_set_color(LED_COLOR_OFF);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        break;
    case POWER_DOMAIN_TCXO:
        // Turn TCXO off.
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_RADIO:
        // Release attached drivers.
        rfe_status = RFE_de_init();
        _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
        sx126x_status = SX126X_de_init();
        _POWER_stack_driver_error(sx126x_status, SX126X_SUCCESS, ERROR_BASE_SX1261, POWER_ERROR_DRIVER_SX126X);
        // Turn radio off.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
        // Turn LED off.
        led_status = LED_set_color(LED_COLOR_OFF);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        break;
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
errors:
    return;
}

/*******************************************************************/
uint8_t POWER_get_state(POWER_domain_t domain) {
    // Local variables.
    uint8_t state = 0;
    // Check parameters.
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    state = (power_domain_state[domain] == 0) ? 0 : 1;
errors:
    return state;
}
