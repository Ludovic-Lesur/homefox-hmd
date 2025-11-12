/*
 * button.c
 *
 *  Created on: 06 oct. 2025
 *      Author: Ludo
 */

#include "button.h"

#include "exti.h"
#include "gpio.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"

/*** BUTTON functions ***/

/*******************************************************************/
BUTTON_status_t BUTTON_init(EXTI_gpio_irq_cb_t irq_callback) {
    // Local variables.
    BUTTON_status_t status = BUTTON_SUCCESS;
    // Configure GPIO.
    EXTI_configure_gpio(&GPIO_BUTTON, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, irq_callback, NVIC_PRIORITY_BUTTON);
    return status;
}

/*******************************************************************/
BUTTON_status_t BUTTON_de_init(void) {
    // Local variables.
    BUTTON_status_t status = BUTTON_SUCCESS;
    // Configure GPIO.
    EXTI_release_gpio(&GPIO_BUTTON, GPIO_MODE_ANALOG);
    return status;
}

/*******************************************************************/
BUTTON_status_t BUTTON_enable_interrupt(void) {
    // Local variables.
    BUTTON_status_t status = BUTTON_SUCCESS;
    // Configure GPIO.
    EXTI_clear_gpio_flag(&GPIO_BUTTON);
    EXTI_enable_gpio_interrupt(&GPIO_BUTTON);
    return status;
}

/*******************************************************************/
BUTTON_status_t BUTTON_disable_interrupt(void) {
    // Local variables.
    BUTTON_status_t status = BUTTON_SUCCESS;
    // Configure GPIO.
    EXTI_disable_gpio_interrupt(&GPIO_BUTTON);
    return status;
}
