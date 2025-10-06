/*
 * button.h
 *
 *  Created on: 06 oct. 2025
 *      Author: Ludo
 */

#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "error.h"
#include "exti.h"
#include "types.h"

/*** BUTTON structures ***/

/*!******************************************************************
 * \enum BUTTON_status_t
 * \brief BUTTON driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    BUTTON_SUCCESS,
    // Last base value.
    BUTTON_ERROR_BASE_LAST = ERROR_BASE_STEP
} BUTTON_status_t;

/*** BUTTON functions ***/

/*!******************************************************************
 * \fn BUTTON_status_t BUTTON_init(void)
 * \brief Init BUTTON driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
BUTTON_status_t BUTTON_init(EXTI_gpio_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn BUTTON_status_t BUTTON_de_init(void)
 * \brief Release BUTTON driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
BUTTON_status_t BUTTON_de_init(void);

/*******************************************************************/
#define BUTTON_exit_error(base) { ERROR_check_exit(led_status, BUTTON_SUCCESS, base) }

/*******************************************************************/
#define BUTTON_stack_error(base) { ERROR_check_stack(led_status, BUTTON_SUCCESS, base) }

/*******************************************************************/
#define BUTTON_stack_exit_error(base, code) { ERROR_check_stack_exit(led_status, BUTTON_SUCCESS, base, code) }

#endif /* __BUTTON_H__ */
