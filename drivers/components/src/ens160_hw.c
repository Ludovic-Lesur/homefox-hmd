/*
 * ens160_hw.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "ens16x_hw.h"

#ifndef ENS16X_DRIVER_DISABLE_FLAGS_FILE
#include "ens16x_driver_flags.h"
#endif

#ifndef ENS16X_DRIVER_DISABLE

#include "ens16x.h"
#include "error_base.h"
#include "sensors_hw.h"
#include "types.h"

/*** ENS16X HW functions ***/

/*******************************************************************/
ENS16X_status_t ENS16X_HW_init(void) {
    return ((ENS16X_status_t) SENSORS_HW_init(ENS16X_ERROR_BASE_I2C));
}

/*******************************************************************/
ENS16X_status_t ENS16X_HW_de_init(void) {
    return ((ENS16X_status_t) SENSORS_HW_de_init(ERROR_BASE_ENS160 + ENS16X_ERROR_BASE_I2C));
}

/*******************************************************************/
ENS16X_status_t ENS16X_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    return ((ENS16X_status_t) SENSORS_HW_i2c_write(ENS16X_ERROR_BASE_I2C, i2c_address, data, data_size_bytes, stop_flag));
}

/*******************************************************************/
ENS16X_status_t __attribute__((weak)) ENS16X_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    return ((ENS16X_status_t) SENSORS_HW_i2c_read(ENS16X_ERROR_BASE_I2C, i2c_address, data, data_size_bytes));
}

#endif /* ENS16X_DRIVER_DISABLE */
