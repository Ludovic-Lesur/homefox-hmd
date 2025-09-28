/*!*****************************************************************
 * \file    rf_api.c
 * \brief   Radio drivers.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/rf_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "manuf/mcu_api.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "power.h"
#include "pwr.h"
#include "rfe.h"
#include "sx126x.h"
#include "tim.h"
#include "types.h"

/*** RF API local macros ***/

#define RF_API_FREQUENCY_MIN_MHZ                863
#define RF_API_FREQUENCY_MAX_MHZ                870

#define RF_API_TCXO_TIMEOUT_MS                  10

#define RF_API_RADIO_SEND_START_LATENCY_BITS    2
#define RF_API_RADIO_SEND_STOP_LATENCY_BITS     9

#define RF_API_TIMER_CHANNEL                    TIM_CHANNEL_4
#define RF_API_TX_TIMEOUT_MS                    3000

#ifdef SIGFOX_EP_BIDIRECTIONAL
#define RF_API_DL_PR_SIZE_BITS                  16
#endif

/*** RF API local structures ***/

/*******************************************************************/
typedef enum {
    // Driver errors.
    RF_API_ERROR_NULL_PARAMETER = (RF_API_SUCCESS + 1),
    RF_API_ERROR_BUFFER_SIZE,
    RF_API_ERROR_STATE,
    RF_API_ERROR_MODULATION,
    RF_API_ERROR_BIT_RATE,
    RF_API_ERROR_MODE,
    RF_API_ERROR_TX_TIMEOUT,
    RF_API_ERROR_LATENCY_TYPE,
    // Low level drivers errors.
    RF_API_ERROR_DRIVER_MCU_API,
    RF_API_ERROR_DRIVER_SX126X,
    RF_API_ERROR_DRIVER_RFE,
    RF_API_ERROR_DRIVER_TIM
} RF_API_custom_status_t;

/*******************************************************************/
typedef union {
    struct {
        unsigned dio_irq_enable :1;
        unsigned dio_irq_flag :1;
    } field;
    sfx_u8 all;
} RF_API_flags_t;

/*******************************************************************/
typedef struct {
    volatile RF_API_flags_t flags;
    // TX.
    sfx_u16 bit_rate_bps;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // RX.
    sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm;
#endif
} RF_API_context_t;

/*** RF API local global variables ***/

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
static sfx_u32 RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
    POWER_ON_DELAY_MS_TCXO, // Wake-up.
    (POWER_ON_DELAY_MS_RADIO + SX126X_EXIT_RESET_DELAY_MS + SX126X_CALIBRATION_DELAY_MS + (RF_API_TCXO_TIMEOUT_MS << 1) + 1), // TX init.
    0, // Send start (depends on bit rate and will be computed during init function).
    0, // Send stop (depends on bit rate and will be computed during init function).
    0, // TX de-init (100us).
    0, // Sleep.
#ifdef SIGFOX_EP_BIDIRECTIONAL
    (POWER_ON_DELAY_MS_RADIO + SX126X_EXIT_RESET_DELAY_MS + SX126X_CALIBRATION_DELAY_MS + (RF_API_TCXO_TIMEOUT_MS << 1) + 1), // RX init.
    0, // Receive start.
    5, // Receive stop.
    0, // RX de-init (100us).
#endif
};
#endif
static RF_API_context_t rf_api_ctx;

/*** RF API local functions ***/

/*******************************************************************/
static void _RF_API_sx126x_dio_irq_callback(void) {
    // Set flag if IRQ is enabled.
    rf_api_ctx.flags.field.dio_irq_flag = rf_api_ctx.flags.field.dio_irq_enable;
}

/*******************************************************************/
static RF_API_status_t _RF_API_enable_sx126x_dio_irq(sfx_u16 irq_mask) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    // Configure interrupt on SX126X side.
    sx126x_status = SX126X_set_dio_irq_mask(irq_mask, irq_mask, 0, 0);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Configure interrupt on MCU side.
    EXTI_configure_gpio(&GPIO_SX1261_DIO1, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, &_RF_API_sx126x_dio_irq_callback, NVIC_PRIORITY_SIGFOX_RADIO_IRQ_GPIO);
    EXTI_clear_gpio_flag(&GPIO_SX1261_DIO1);
    // Enable interrupt.
    rf_api_ctx.flags.field.dio_irq_enable = 1;
    EXTI_enable_gpio_interrupt(&GPIO_SX1261_DIO1);
errors:
    return status;
}

/*******************************************************************/
static void _RF_API_disable_sx126x_dio_irq(void) {
    // Local variables.
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    // Configure interrupt on SX126X side.
    sx126x_status = SX126X_set_dio_irq_mask(0, 0, 0, 0);
    SX126X_stack_error(ERROR_BASE_SX1261);
    // Disable GPIO interrupt.
    rf_api_ctx.flags.field.dio_irq_enable = 0;
    EXTI_disable_gpio_interrupt(&GPIO_SX1261_DIO1);
    EXTI_release_gpio(&GPIO_SX1261_DIO1, GPIO_MODE_INPUT);
}

/*** RF API functions ***/

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t* rf_api_config) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Ignore unused parameters.
    UNUSED(rf_api_config);
    // Return.
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Turn radio TCXO on.
    POWER_enable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_TCXO, LPTIM_DELAY_MODE_SLEEP);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Turn radio TCXO off.
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_TCXO);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t* radio_parameters) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    SX126X_modulation_parameters_t modulation_parameters;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_u8 dl_ft[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
    SX126X_gfsk_packet_parameters_t gfsk_packet_parameters;
    sfx_u8 idx = 0;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (radio_parameters == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Turn radio on.
    POWER_enable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO, LPTIM_DELAY_MODE_SLEEP);
    // Exit reset.
    sx126x_status = SX126X_reset(0);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Regulation mode.
    sx126x_status = SX126X_set_regulation_mode(SX126X_REGULATION_MODE_DCDC);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Oscillator.
    sx126x_status = SX126X_set_oscillator(SX126X_OSCILLATOR_TCXO, SX126X_TCXO_VOLTAGE_1V6, RF_API_TCXO_TIMEOUT_MS);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Calibration.
    sx126x_status = SX126X_calibrate(RF_API_FREQUENCY_MIN_MHZ, RF_API_FREQUENCY_MAX_MHZ);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Frequency.
    sx126x_status = SX126X_set_rf_frequency(radio_parameters->frequency_hz);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Update context.
    rf_api_ctx.bit_rate_bps = (radio_parameters->bit_rate_bps);
    // Common modulation parameters.
#ifdef SIGFOX_EP_BIDIRECTIONAL
    modulation_parameters.fsk_deviation_hz = (radio_parameters->deviation_hz);
#else
    modulation_parameters.fsk_deviation_hz = 0;
#endif
    modulation_parameters.bit_rate_bps = (radio_parameters->bit_rate_bps);
    modulation_parameters.rx_bandwidth = SX126X_RXBW_4800HZ;
    // Modulation parameters.
    switch (radio_parameters->modulation) {
    case RF_API_MODULATION_NONE:
        modulation_parameters.modulation = SX126X_MODULATION_GFSK;
        modulation_parameters.modulation_shaping = SX126X_MODULATION_SHAPING_NONE;
        break;
    case RF_API_MODULATION_DBPSK:
        modulation_parameters.modulation = SX126X_MODULATION_BPSK;
        modulation_parameters.modulation_shaping = SX126X_MODULATION_SHAPING_DBPSK;

        break;
    case RF_API_MODULATION_GFSK:
        modulation_parameters.modulation = SX126X_MODULATION_GFSK;
        modulation_parameters.modulation_shaping = SX126X_MODULATION_SHAPING_GAUSSIAN_BT_1;
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODULATION);
        break;
    }
    // Set modulation scheme.
    sx126x_status = SX126X_set_modulation(&modulation_parameters);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Configure specific registers.
    switch (radio_parameters->rf_mode) {
    case RF_API_MODE_TX:
        // Set output power.
        sx126x_status = SX126X_set_rf_output_power((radio_parameters->tx_power_dbm_eirp), SX126X_PA_RAMP_TIME_40U);
        SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
        // Start latency = ramp-up.
        RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = (((RF_API_RADIO_SEND_START_LATENCY_BITS * 1000) / ((sfx_u32) (radio_parameters->bit_rate_bps))) + RF_API_TCXO_TIMEOUT_MS);
        // Stop latency = ramp-down + half of padding bit (since IRQ is raised at the middle of the symbol).
        RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] = (((RF_API_RADIO_SEND_STOP_LATENCY_BITS * 1000) / ((sfx_u32) (radio_parameters->bit_rate_bps))) + RF_API_TCXO_TIMEOUT_MS);
#endif
        // Switch to TX.
        rfe_status = RFE_set_path(RFE_PATH_TX);
        RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RF_API_MODE_RX:
        // LNA mode.
        sx126x_status = SX126X_set_lna_mode(SX126X_LNA_MODE_BOOST);
        SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
        // Downlink packet structure.
        gfsk_packet_parameters.preamble_length_bits = RF_API_DL_PR_SIZE_BITS;
        gfsk_packet_parameters.preamble_detector_length = SX126X_PREAMBLE_DETECTOR_LENGTH_16BITS;
        for (idx = 0; idx < SIGFOX_DL_FT_SIZE_BYTES; idx++) {
            gfsk_packet_parameters.sync_word[idx] = dl_ft[idx];
        }
        gfsk_packet_parameters.sync_word_length_bits = (SIGFOX_DL_FT_SIZE_BYTES << 3);
        gfsk_packet_parameters.payload_length_bytes = SIGFOX_DL_PHY_CONTENT_SIZE_BYTES;
        sx126x_status = SX126X_set_gfsk_packet(&gfsk_packet_parameters);
        SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
        // Switch to RX.
        rfe_status = RFE_set_path(RFE_PATH_RX);
        RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODE);
        break;
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Disable front-end.
    rfe_status = RFE_set_path(RFE_PATH_NONE);
    // Check status.
    if (rfe_status != RFE_SUCCESS) {
        RFE_stack_error(ERROR_BASE_RFE);
        status = (RF_API_status_t) RF_API_ERROR_DRIVER_RFE;
    }
    // Turn transceiver off.
    sx126x_status = SX126X_reset(1);
    // Check status.
    if (sx126x_status != SX126X_SUCCESS) {
        SX126X_stack_error(ERROR_BASE_SX1261);
        status = (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X;
    }
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t* tx_data) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    SX126X_bpsk_packet_parameters_t bpsk_packet_parameters;
    sfx_u8 buffer[SIGFOX_UL_BITSTREAM_SIZE_BYTES + 1];
    sfx_u8 differential_size_bytes = 0;
    sfx_u16 differential_size_bits = 0;
    sfx_u8 timer_has_elapsed = 0;
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (tx_data == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Reset flags.
    rf_api_ctx.flags.all = 0;
    // Differential encoding.
    for (idx = 0; idx < (tx_data->bitstream_size_bytes); idx++) {
        buffer[idx] = tx_data->bitstream[idx];
    }
    buffer[tx_data->bitstream_size_bytes] = 0x80;
    sx126x_status = SX126X_differential_encoding(buffer, (tx_data->bitstream_size_bytes), buffer, &differential_size_bytes, &differential_size_bits);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Set packet parameters.
    bpsk_packet_parameters.payload_length_bytes = differential_size_bytes;
    bpsk_packet_parameters.payload_length_bits = differential_size_bits;
    // Ramp times.
    switch (rf_api_ctx.bit_rate_bps) {
    case 100:
        bpsk_packet_parameters.ramp_up_delay = SX126X_RAMP_UP_DELAY_DBPSK_100BPS;
        bpsk_packet_parameters.ramp_down_delay = SX126X_RAMP_DOWN_DELAY_DBPSK_100BPS;
        break;
    case 600:
        bpsk_packet_parameters.ramp_up_delay = SX126X_RAMP_UP_DELAY_DBPSK_600BPS;
        bpsk_packet_parameters.ramp_down_delay = SX126X_RAMP_DOWN_DELAY_DBPSK_600BPS;
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_BIT_RATE);
    }
    sx126x_status = SX126X_set_bpsk_packet(&bpsk_packet_parameters);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Write FIFO.
    sx126x_status = SX126X_write_fifo(buffer, differential_size_bytes);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Enable GPIO interrupt.
    status = _RF_API_enable_sx126x_dio_irq(0b1 << SX126X_IRQ_INDEX_TX_DONE);
    if (status != RF_API_SUCCESS) goto errors;
    // Start timer (TIM MCH has been initialized by the MCU_API
    tim_status = TIM_MCH_start_channel(TIM_INSTANCE_MCU_API, RF_API_TIMER_CHANNEL, RF_API_TX_TIMEOUT_MS, TIM_WAITING_MODE_ACTIVE);
    TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (RF_API_status_t) RF_API_ERROR_DRIVER_TIM);
    // Start transmission.
    sx126x_status = SX126X_set_mode(SX126X_MODE_TX);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Wait for transmission to complete.
    while (rf_api_ctx.flags.field.dio_irq_flag == 0) {
        // Enter sleep mode.
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        // Check timer.
        tim_status = TIM_MCH_get_channel_status(TIM_INSTANCE_MCU_API, RF_API_TIMER_CHANNEL, &timer_has_elapsed);
        TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (RF_API_status_t) RF_API_ERROR_DRIVER_TIM);
        // Exit if timeout.
        if (timer_has_elapsed != 0) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_TX_TIMEOUT);
        }
    }
errors:
    // Stop timer and disable GPIO interrupt.
    TIM_MCH_stop_channel(TIM_INSTANCE_MCU_API, RF_API_TIMER_CHANNEL);
    _RF_API_disable_sx126x_dio_irq();
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t* rx_data) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    sfx_bool dl_timeout = SIGFOX_FALSE;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (rx_data == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Reset flags.
    rf_api_ctx.flags.all = 0;
    (rx_data->data_received) = SIGFOX_FALSE;
    // Enable GPIO interrupt.
    status = _RF_API_enable_sx126x_dio_irq(0b1 << SX126X_IRQ_INDEX_RX_DONE);
    if (status != RF_API_SUCCESS) goto errors;
    // Start reception.
    sx126x_status = SX126X_set_mode(SX126X_MODE_RX);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Wait for GPIO interrupt.
    while (rf_api_ctx.flags.field.dio_irq_flag == 0) {
        // Enter sleep mode.
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        IWDG_reload();
        // Check timeout.
        mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
        MCU_API_check_status((RF_API_status_t) RF_API_ERROR_DRIVER_MCU_API);
        // Exit if timeout.
        if (dl_timeout == SIGFOX_TRUE) {
            // Stop radio.
            sx126x_status = SX126X_set_mode(SX126X_MODE_STANDBY_XOSC);
            SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
            // Exit loop.
            goto errors;
        }
    }
    // Read data.
    sx126x_status = SX126X_read_fifo(rf_api_ctx.dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
    // Read RSSI.
    rfe_status = RFE_get_rssi(SX126X_RSSI_TYPE_SYNC_WORD, &(rf_api_ctx.dl_rssi_dbm));
    RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
    // Update status flag.
    (rx_data->data_received) = SIGFOX_TRUE;
errors:
    // Disable GPIO interrupt.
    _RF_API_disable_sx126x_dio_irq();
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8* dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16* dl_rssi_dbm) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if ((dl_phy_content == SIGFOX_NULL) || (dl_rssi_dbm == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
    if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_BUFFER_SIZE);
    }
#endif
    // Fill data.
    for (idx = 0; idx < dl_phy_content_size; idx++) {
        dl_phy_content[idx] = rf_api_ctx.dl_phy_content[idx];
    }
    (*dl_rssi_dbm) = (sfx_s16) rf_api_ctx.dl_rssi_dbm;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_UNUSED(carrier_sense_params);
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32* latency_ms) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (latency_type >= RF_API_LATENCY_LAST) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_LATENCY_TYPE);
    }
#endif
    // Set latency.
    (*latency_ms) = RF_API_LATENCY_MS[latency_type];
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SX126X_status_t sx126x_status = SX126X_SUCCESS;
    // Start continuous wave.
    sx126x_status = SX126X_set_mode(SX126X_MODE_TX_CW);
    SX126X_stack_exit_error(ERROR_BASE_SX1261, (RF_API_status_t) RF_API_ERROR_DRIVER_SX126X);
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_UNUSED(version);
    SIGFOX_UNUSED(version_size_char);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
    // Force all front-end off.
    SX126X_reset(1);
    RFE_set_path(RFE_PATH_NONE);
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_TCXO);
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO);
}
#endif
