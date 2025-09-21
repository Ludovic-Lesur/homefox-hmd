/*
 * sx126x_driver_flags.h
 *
 *  Created on: 14 sep. 2025
 *      Author: Ludo
 */

#ifndef __SX126X_DRIVER_FLAGS_H__
#define __SX126X_DRIVER_FLAGS_H__

#include "lptim.h"
#include "spi.h"

/*** SX126X driver compilation flags ***/

#define SX126X_DRIVER_SPI_ERROR_BASE_LAST       SPI_ERROR_BASE_LAST
#define SX126X_DRIVER_DELAY_ERROR_BASE_LAST     LPTIM_ERROR_BASE_LAST

//#define SX126X_DRIVER_DEVICE_SX1262

#define SX126X_DRIVER_FXOSC_HZ                  32000000

#define SX126X_DRIVER_TX_ENABLE
#define SX126X_DRIVER_RX_ENABLE

#endif /* __SX126X_DRIVER_FLAGS_H__ */
