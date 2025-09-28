/*
 * sx126x_hw.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "sx126x_hw.h"

#ifndef SX126X_DRIVER_DISABLE_FLAGS_FILE
#include "sx126x_driver_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "spi.h"
#include "sx126x.h"
#include "types.h"

#ifndef SX126X_DRIVER_DISABLE

/*** SX126X HW local macros ***/

#define SX126X_HW_BUSY_TIMEOUT_COUNT    1000000

/*** SX126X HW functions ***/

/*******************************************************************/
SX126X_status_t SX126X_HW_init(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    SPI_configuration_t spi_config;
    // Configure reset pin.
    GPIO_configure(&GPIO_SX1261_NRESET, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_SX1261_NRESET, 0);
    // Configure busy pin.
    GPIO_configure(&GPIO_SX1261_BUSY, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    // Init SPI.
    spi_config.baud_rate_prescaler = SPI_BAUD_RATE_PRESCALER_4;
    spi_config.data_format = SPI_DATA_FORMAT_8_BITS;
    spi_config.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_status = SPI_init(SPI_INSTANCE_RADIO, &SPI_GPIO_SX1261, &spi_config);
    SPI_exit_error(SX126X_ERROR_BASE_SPI);
    // Configure chip select pin.
    GPIO_configure(&GPIO_SX1261_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_SX1261_CS, 1);
    errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_HW_de_init(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // Keep reset pin as output low.
    GPIO_write(&GPIO_SX1261_NRESET, 0);
    // Switch busy pin to output low.
    GPIO_configure(&GPIO_SX1261_BUSY, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_SX1261_BUSY, 0);
    // Keep chip select pin as output low.
    GPIO_write(&GPIO_SX1261_CS, 0);
    // Release SPI.
    spi_status = SPI_de_init(SPI_INSTANCE_RADIO, &SPI_GPIO_SX1261);
    SPI_stack_error(ERROR_BASE_SX1261 + SX126X_ERROR_BASE_SPI);
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_HW_set_nreset_gpio(uint8_t state) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    // Set GPIO state.
    GPIO_write(&GPIO_SX1261_NRESET, state);
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_HW_wait_busy_low(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint32_t loop_count = 0;
    // Wait for busy pin to be low.
    while (GPIO_read(&GPIO_SX1261_BUSY) != 0) {
        // Exit of timeout.
        loop_count++;
        if (loop_count > SX126X_HW_BUSY_TIMEOUT_COUNT) {
            status = SX126X_ERROR_BUSY_TIMEOUT;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    SPI_status_t spi_status = SPI_SUCCESS;
    // CS low.
    GPIO_write(&GPIO_SX1261_CS, 0);
    // SPI transfer.
    spi_status = SPI_write_read_8(SPI_INSTANCE_RADIO, tx_data, rx_data, transfer_size);
    SPI_exit_error(SX126X_ERROR_BASE_SPI);
errors:
    // CS high.
    GPIO_write(&GPIO_SX1261_CS, 1);
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Perform delay.
    lptim_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    LPTIM_exit_error(SX126X_ERROR_BASE_DELAY);
errors:
    return status;
}

#endif /* SX126X_DRIVER_DISABLE */
