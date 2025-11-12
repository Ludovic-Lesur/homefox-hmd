/*
 * hmd_flags.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __HMD_FLAGS_H__
#define __HMD_FLAGS_H__

#include "types.h"

/*** Board modes ***/

//#define HMD_MODE_CLI
//#define HMD_MODE_DEBUG

/*** Board parameters ***/

#define HMD_BUTTON_ENABLE
#define HMD_SHT3X_ENABLE
#define HMD_ENS21X_ENABLE
#define HMD_ENS16X_ENABLE
#define HMD_FXLS89XXXX_ENABLE

#define HMD_MONITORING_PERIOD_SECONDS           3600
#ifdef HMD_ENS16X_ENABLE
#define HMD_AIR_QUALITY_PERIOD_SECONDS          3600
#endif
#ifdef HMD_FXLS89XXXX_ENABLE
#define HMD_FXLS89XXXX_BLANKING_TIME_SECONDS    60
#endif

#endif /* __HMD_FLAGS_H__ */
