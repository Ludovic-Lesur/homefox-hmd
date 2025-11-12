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
 * \brief Init push button driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
BUTTON_status_t BUTTON_init(EXTI_gpio_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn BUTTON_status_t BUTTON_de_init(void)
 * \brief Release push button driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
BUTTON_status_t BUTTON_de_init(void);

/*!******************************************************************
 * \fn BUTTON_status_t BUTTON_enable_interrupt(void)
 * \brief Enable push button interrupt.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
BUTTON_status_t BUTTON_enable_interrupt(void);

/*!******************************************************************
 * \fn BUTTON_status_t BUTTON_disable_interrupt(void)
 * \brief Disable push button interrupt.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
BUTTON_status_t BUTTON_disable_interrupt(void);

/*******************************************************************/
#define BUTTON_exit_error(base) { ERROR_check_exit(button_status, BUTTON_SUCCESS, base) }

/*******************************************************************/
#define BUTTON_stack_error(base) { ERROR_check_stack(button_status, BUTTON_SUCCESS, base) }

/*******************************************************************/
#define BUTTON_stack_exit_error(base, code) { ERROR_check_stack_exit(button_status, BUTTON_SUCCESS, base, code) }

#endif /* __BUTTON_H__ */
