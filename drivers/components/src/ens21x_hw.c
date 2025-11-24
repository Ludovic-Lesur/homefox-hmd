/*
 * ens21x_hw.c
 *
 *  Created on: 19 oct. 2025
 *      Author: Ludo
 */

#include "ens21x_hw.h"

#ifndef ENS21X_DRIVER_DISABLE_FLAGS_FILE
#include "ens21x_driver_flags.h"
#endif
#include "error_base.h"
#include "sensors_hw.h"
#include "ens21x.h"
#include "types.h"

#ifndef ENS21X_DRIVER_DISABLE

/*** ENS21X HW functions ***/

/*******************************************************************/
ENS21X_status_t ENS21X_HW_init(void) {
    return ((ENS21X_status_t) SENSORS_HW_init(ENS21X_ERROR_BASE_I2C));
}

/*******************************************************************/
ENS21X_status_t ENS21X_HW_de_init(void) {
    return ((ENS21X_status_t) SENSORS_HW_de_init(ERROR_BASE_SHT30 + ENS21X_ERROR_BASE_I2C));
}

/*******************************************************************/
ENS21X_status_t ENS21X_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((ENS21X_status_t) SENSORS_HW_i2c_write(ENS21X_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
ENS21X_status_t ENS21X_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((ENS21X_status_t) SENSORS_HW_i2c_read(ENS21X_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

/*******************************************************************/
ENS21X_status_t ENS21X_HW_delay_milliseconds(uint32_t delay_ms) {
    return ((ENS21X_status_t) SENSORS_HW_delay_milliseconds(ENS21X_ERROR_BASE_DELAY, delay_ms));
}

#endif /* ENS21X_DRIVER_DISABLE */
