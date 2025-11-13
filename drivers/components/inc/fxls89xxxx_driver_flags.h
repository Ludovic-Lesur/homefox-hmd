/*
 * fxls89xxxx_driver_flags.h
 *
 *  Created on: 17 oct. 2025
 *      Author: Ludo
 */

#ifndef __FXLS89XXXX_DRIVER_FLAGS_H__
#define __FXLS89XXXX_DRIVER_FLAGS_H__

#include "hmd_flags.h"
#include "i2c.h"

/*** FXLS89XXXX driver compilation flags ***/

#ifndef HMD_ACCELEROMETER_ENABLE
#define FXLS89XXXX_DRIVER_DISABLE
#endif

#define FXLS89XXXX_DRIVER_I2C_ERROR_BASE_LAST   I2C_ERROR_BASE_LAST

#endif /* __FXLS89XXXX_DRIVER_FLAGS_H__ */
