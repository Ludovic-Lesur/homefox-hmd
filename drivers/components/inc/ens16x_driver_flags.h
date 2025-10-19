/*
 * ens16x_driver_flags.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __ENS16X_DRIVER_FLAGS_H__
#define __ENS16X_DRIVER_FLAGS_H__

#include "hmd_flags.h"
#include "i2c.h"

/*** ENS16X driver compilation flags ***/

#ifndef HMD_ENS16X_ENABLE
#define ENS16X_DRIVER_DISABLE
#endif

#define ENS16X_DRIVER_I2C_ERROR_BASE_LAST   I2C_ERROR_BASE_LAST

//#define ENS16X_DRIVER_DEVICE_ENS161

#endif /* __ENS16X_DRIVER_FLAGS_H__ */
