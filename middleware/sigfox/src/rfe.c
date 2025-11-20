/*
 * rfe.c
 *
 *  Created on: 28 sep. 2025
 *      Author: Ludo
 */

#include "rfe.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "mcu_mapping.h"
#include "sx126x.h"
#include "types.h"

/*** RFE local macros ***/

#define RFE_RX_OFFSET_DB    0

/*** RFE functions ***/

/*******************************************************************/
RFE_status_t RFE_init(void) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    // Configure GPIOs.
#ifdef HW1_0
    GPIO_configure(&GPIO_RF_RX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_RX_ENABLE, 0);
#endif
    GPIO_configure(&GPIO_RF_TX_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_TX_ENABLE, 0);
    GPIO_configure(&GPIO_RF_LNA_BYPASS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_LNA_BYPASS, 0);
    return status;
}

/*******************************************************************/
RFE_status_t RFE_de_init(void) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Set all pins to output low.
    rfe_status = RFE_set_path(RFE_PATH_NONE);
    RFE_stack_error(ERROR_BASE_RFE);
    return status;
}

/*******************************************************************/
RFE_status_t RFE_set_path(RFE_path_t radio_path) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    switch (radio_path) {
    case RFE_PATH_NONE:
#ifdef HW1_0
        GPIO_write(&GPIO_RF_RX_ENABLE, 0);
#endif
#ifdef HW2_0
        GPIO_write(&GPIO_RF_TX_ENABLE, 0);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 0);
#endif
        break;
    case RFE_PATH_TX:
#ifdef HW1_0
        GPIO_write(&GPIO_RF_RX_ENABLE, 0);
#endif
#ifdef HW2_0
        GPIO_write(&GPIO_RF_TX_ENABLE, 1);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 1);
#endif
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RFE_PATH_RX_BYPASS:
#ifdef HW1_0
        GPIO_write(&GPIO_RF_RX_ENABLE, 1);
#endif
#ifdef HW2_0
        GPIO_write(&GPIO_RF_TX_ENABLE, 0);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 1);
#endif
        break;
#ifdef HW2_0
    case RFE_PATH_RX_LNA:
        GPIO_write(&GPIO_RF_TX_ENABLE, 0);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 0);
        break;
#endif
#endif
    default:
        status = RFE_ERROR_PATH;
        goto errors;
    }
errors:
    return status;
}

#if ((defined SIGFOX_EP_BIDIRECTIONAL) && !(defined SX126X_DRIVER_DISABLE))
/*******************************************************************/
RFE_status_t RFE_get_rssi(SX126X_rssi_t rssi_type, int16_t* rssi_dbm) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    // Read raw RSSI.
    sx126x_status = SX126X_get_rssi(rssi_type, rssi_dbm);
    SX126X_exit_error(RFE_ERROR_BASE_SX126X);
    // Apply calibration gain.
    (*rssi_dbm) += RFE_RX_OFFSET_DB;
errors:
    return status;
}
#endif
