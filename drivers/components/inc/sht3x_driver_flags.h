/*
 * sht3x_driver_flags.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __SHT3X_DRIVER_FLAGS_H__
#define __SHT3X_DRIVER_FLAGS_H__

#include "hmd_flags.h"
#include "i2c.h"
#include "lptim.h"

/*** SHT3X driver compilation flags ***/

#ifndef HMD_SHT3X_ENABLE
#define SHT3X_DRIVER_DISABLE
#endif

#define SHT3X_DRIVER_I2C_ERROR_BASE_LAST    I2C_ERROR_BASE_LAST
#define SHT3X_DRIVER_DELAY_ERROR_BASE_LAST  LPTIM_ERROR_BASE_LAST

#endif /* __SHT3X_DRIVER_FLAGS_H__ */
