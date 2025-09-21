/*
 * nvic_priority.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __NVIC_PRIORITY_H__
#define __NVIC_PRIORITY_H__

/*!******************************************************************
 * \enum NVIC_priority_list_t
 * \brief NVIC interrupt priorities list.
 *******************************************************************/
typedef enum {
    // Common.
    NVIC_PRIORITY_CLOCK = 0,
    NVIC_PRIORITY_CLOCK_CALIBRATION = 1,
    NVIC_PRIORITY_DELAY = 2,
    NVIC_PRIORITY_RTC = 3,
    // Sensor.
    NVIC_PRIORITY_SENSOR = 0,
    // Sigfox.
    NVIC_PRIORITY_SIGFOX_TIMER = 1,
    // AT interface.
    NVIC_PRIORITY_AT = 3
} NVIC_priority_list_t;

#endif /* __NVIC_PRIORITY_H__ */
