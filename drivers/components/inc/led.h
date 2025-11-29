/*
 * led.h
 *
 *  Created on: 29 sep. 2025
 *      Author: Ludo
 */

#ifndef __LED_H__
#define __LED_H__

#include "error.h"
#include "hmd_flags.h"
#include "types.h"

/*** LED structures ***/

/*!******************************************************************
 * \enum LED_status_t
 * \brief LED driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    LED_SUCCESS,
    LED_ERROR_COLOR,
    LED_ERROR_ACTIVITY,
    // Last base value.
    LED_ERROR_BASE_LAST = ERROR_BASE_STEP
} LED_status_t;

/*!******************************************************************
 * \enum LED_color_t
 * \brief LED colors list.
 *******************************************************************/
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_YELLOW,
    LED_COLOR_BLUE,
    LED_COLOR_MAGENTA,
    LED_COLOR_CYAN,
    LED_COLOR_WHITE,
    LED_COLOR_LAST
} LED_color_t;

/*!******************************************************************
 * \enum LED_activity_t
 * \brief LED activity list.
 *******************************************************************/
typedef enum {
    LED_ACTIVITY_NONE = 0,
    LED_ACTIVITY_SIGFOX_UPLINK,
    LED_ACTIVITY_SIGFOX_DOWNLINK,
    LED_ACTIVITY_TEMPERATURE_HUMIDITY_READING,
#ifdef HMD_AIR_QUALITY_ENABLE
    LED_ACTIVITY_AIR_QUALITY_READING,
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
    LED_ACTIVITY_ACCELEROMETER_READING,
#endif
    LED_ACTIVITY_LAST
} LED_activity_t;

/*** LED functions ***/

/*!******************************************************************
 * \fn LED_status_t LED_init(void)
 * \brief Init LED driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_init(void);

/*!******************************************************************
 * \fn LED_status_t LED_de_init(void)
 * \brief Release LED driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_de_init(void);

/*!******************************************************************
 * \fn LED_status_t LED_set_color(LED_color_t led_color)
 * \brief Set LED color.
 * \param[in]   led_color: Color to select.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_set_color(LED_color_t led_color);

/*!******************************************************************
 * \fn LED_status_t LED_set_activity_color(LED_activity_t led_activity, LED_color_t led_color)
 * \brief Set LED activity color.
 * \param[in]   led_activity: Activity to configure.
 * \param[in]   led_color: Color to use during given activity.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_set_activity_color(LED_activity_t led_activity, LED_color_t led_color);

/*!******************************************************************
 * \fn LED_status_t LED_set_activity(LED_activity_t led_activity)
 * \brief Set LED color according to activity.
 * \param[in]   led_activity: Activity to notify.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_set_activity(LED_activity_t led_activity);

/*******************************************************************/
#define LED_exit_error(base) { ERROR_check_exit(led_status, LED_SUCCESS, base) }

/*******************************************************************/
#define LED_stack_error(base) { ERROR_check_stack(led_status, LED_SUCCESS, base) }

/*******************************************************************/
#define LED_stack_exit_error(base, code) { ERROR_check_stack_exit(led_status, LED_SUCCESS, base, code) }

#endif /* __LED_H__ */
