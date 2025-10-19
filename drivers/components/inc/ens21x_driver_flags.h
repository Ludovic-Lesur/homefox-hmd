/*
 * ens21x_driver_flags.h
 *
 *  Created on: 19 oct. 2025
 *      Author: Ludo
 */

#ifndef __ENS21X_DRIVER_FLAGS_H__
#define __ENS21X_DRIVER_FLAGS_H__

#include "hmd_flags.h"
#include "i2c.h"
#include "lptim.h"

/*** ENS21X driver compilation flags ***/

#ifndef HMD_ENS21X_ENABLE
#define ENS21X_DRIVER_DISABLE
#endif

#define ENS21X_DRIVER_I2C_ERROR_BASE_LAST   I2C_ERROR_BASE_LAST
#define ENS21X_DRIVER_DELAY_ERROR_BASE_LAST LPTIM_ERROR_BASE_LAST

#endif /* __ENS21X_DRIVER_FLAGS_H__ */
