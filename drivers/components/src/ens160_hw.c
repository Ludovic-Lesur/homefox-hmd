/*
 * ens160_hw.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "ens160_hw.h"

#ifndef ENS160_DRIVER_DISABLE_FLAGS_FILE
#include "ens160_driver_flags.h"
#endif

#ifndef ENS160_DRIVER_DISABLE

#include "ens160.h"
#include "error_base.h"
#include "sensors_hw.h"
#include "types.h"

/*** ENS160 HW functions ***/

/*******************************************************************/
ENS160_status_t ENS160_HW_init(void) {
    return ((ENS160_status_t) SENSORS_HW_init(ENS160_ERROR_BASE_I2C));
}

/*******************************************************************/
ENS160_status_t ENS160_HW_de_init(void) {
    return ((ENS160_status_t) SENSORS_HW_de_init(ERROR_BASE_ENS160 + ENS160_ERROR_BASE_I2C));
}

/*******************************************************************/
ENS160_status_t ENS160_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((ENS160_status_t) SENSORS_HW_i2c_write(ENS160_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
ENS160_status_t __attribute__((weak)) ENS160_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((ENS160_status_t) SENSORS_HW_i2c_read(ENS160_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

#endif /* ENS160_DRIVER_DISABLE */
