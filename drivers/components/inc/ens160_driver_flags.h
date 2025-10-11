/*
 * ens160_driver_flags.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __ENS160_DRIVER_FLAGS_H__
#define __ENS160_DRIVER_FLAGS_H__

#include "hmd_flags.h"
#include "i2c.h"

/*** ENS160 driver compilation flags ***/

#ifndef HMD_AIR_QUALITY_ENABLE
#define ENS160_DRIVER_DISABLE
#endif

#define ENS160_DRIVER_I2C_ERROR_BASE_LAST   I2C_ERROR_BASE_LAST

#endif /* __ENS160_DRIVER_FLAGS_H__ */
