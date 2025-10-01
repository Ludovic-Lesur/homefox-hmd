/*
 * main.c
 *
 *  Created on: 13 sep. 2025
 *      Author: Ludo
 */

// Drivers.
#include "exti.h"
#include "gpio.h"
#include "i2c_address.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Utils.
#include "error.h"
#include "types.h"
// Components.
#include "led.h"
// Middleware
#include "cli.h"
// Applicative.
#include "error_base.h"
#include "hmd_flags.h"

/*** MAIN local functions ***/

/*******************************************************************/
static void _HMD_rtc_wakeup_timer_irq_callback(void) {
    // TODO
}

/*******************************************************************/
static void _HMD_init_hw(void) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
#ifndef HMD_MODE_DEBUG
    IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
    LED_status_t led_status = LED_SUCCESS;
    // Init error stack
    ERROR_stack_init();
    // Init memory.
    NVIC_init();
    // Init power module and clock tree.
    PWR_init();
    rcc_status = RCC_init(NVIC_PRIORITY_CLOCK);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init GPIOs.
    GPIO_init();
    POWER_init();
    EXTI_init();
    // Start independent watchdog.
#ifndef HMD_MODE_DEBUG
    iwdg_status = IWDG_init();
    IWDG_stack_error(ERROR_BASE_IWDG);
    IWDG_reload();
#endif
    // High speed oscillator.
    rcc_status = RCC_switch_to_hsi();
    RCC_stack_error(ERROR_BASE_RCC);
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init RTC.
    rtc_status = RTC_init(&_HMD_rtc_wakeup_timer_irq_callback, NVIC_PRIORITY_RTC);
    RTC_stack_error(ERROR_BASE_RTC);
    // Init delay timer.
    lptim_status = LPTIM_init(NVIC_PRIORITY_DELAY);
    LPTIM_stack_error(ERROR_BASE_LPTIM);
    // Init RGB LED.
    led_status = LED_init();
    LED_stack_error(ERROR_BASE_LED);
}

/*** MAIN functions ***/

#ifndef HMD_MODE_CLI
/*******************************************************************/
int main(void) {
    // Init board.
    _HMD_init_hw();
    // Main loop.
    PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STANDBY);
    return 0;
}
#endif

#ifdef HMD_MODE_CLI
/*******************************************************************/
int main(void) {
    // Local variables.
    CLI_status_t cli_status = CLI_SUCCESS;
    // Init board.
    _HMD_init_hw();
    // Init command line interface.
    cli_status = CLI_init();
    CLI_stack_error(ERROR_BASE_CLI);
    // Main loop.
    while (1) {
        // Enter sleep mode.
        IWDG_reload();
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        IWDG_reload();
        // Process command line interface.
        cli_status = CLI_process();
        CLI_stack_error(ERROR_BASE_CLI);
    }
    return 0;
}
#endif
