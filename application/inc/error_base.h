/*
 * error.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __ERROR_BASE_H__
#define __ERROR_BASE_H__

// Peripherals.
#include "aes.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvm.h"
#include "rcc.h"
#include "rtc.h"
#include "tim.h"
// Utils.
#include "error.h"
#include "maths.h"
#include "terminal.h"
// Components.
#include "button.h"
#include "ens16x.h"
#include "ens21x.h"
#include "fxls89xxxx.h"
#include "led.h"
#include "sht3x.h"
#include "sx126x.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "power.h"
#include "rfe.h"
#include "sigfox_ep_addon_rfp_api.h"
#include "sigfox_ep_api.h"
// Sigfox.
#include "sigfox_error.h"

/*** ERROR BASE structures ***/

/*!******************************************************************
 * \enum ERROR_base_t
 * \brief Board error bases.
 *******************************************************************/
typedef enum {
    SUCCESS = 0,
    // Peripherals.
    ERROR_BASE_AES = ERROR_BASE_STEP,
    ERROR_BASE_IWDG = (ERROR_BASE_AES + AES_ERROR_BASE_LAST),
    ERROR_BASE_LPTIM = (ERROR_BASE_IWDG + IWDG_ERROR_BASE_LAST),
    ERROR_BASE_NVM = (ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    ERROR_BASE_RCC = (ERROR_BASE_NVM + NVM_ERROR_BASE_LAST),
    ERROR_BASE_RTC = (ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    ERROR_BASE_TIM_MCU_API = (ERROR_BASE_RTC + RTC_ERROR_BASE_LAST),
    ERROR_BASE_TIM_RF_API = (ERROR_BASE_TIM_MCU_API + TIM_ERROR_BASE_LAST),
    // Utils.
    ERROR_BASE_MATH = (ERROR_BASE_TIM_RF_API + TIM_ERROR_BASE_LAST),
    ERROR_BASE_TERMINAL_AT = (ERROR_BASE_MATH + MATH_ERROR_BASE_LAST),
    // Components.
    ERROR_BASE_BUTTON = (ERROR_BASE_TERMINAL_AT + TERMINAL_ERROR_BASE_LAST),
    ERROR_BASE_ENS16X = (ERROR_BASE_BUTTON + BUTTON_ERROR_BASE_LAST),
    ERROR_BASE_ENS210 = (ERROR_BASE_ENS16X + ENS16X_ERROR_BASE_LAST),
    ERROR_BASE_FXLS8974CF = (ERROR_BASE_ENS210 + ENS21X_ERROR_BASE_LAST),
    ERROR_BASE_LED = (ERROR_BASE_FXLS8974CF + FXLS89XXXX_ERROR_BASE_LAST),
    ERROR_BASE_SHT30 = (ERROR_BASE_LED + LED_ERROR_BASE_LAST),
    ERROR_BASE_SX1261 = (ERROR_BASE_SHT30 + SHT3X_ERROR_BASE_LAST),
    // Middleware.
    ERROR_BASE_ANALOG = (ERROR_BASE_SX1261 + SX126X_ERROR_BASE_LAST),
    ERROR_BASE_CLI = (ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_LAST),
    ERROR_BASE_POWER = (ERROR_BASE_CLI + CLI_ERROR_BASE_LAST),
    ERROR_BASE_RFE = (ERROR_BASE_POWER + POWER_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_LIB = (ERROR_BASE_RFE + RFE_ERROR_BASE_LAST),
    ERROR_BASE_SIGFOX_EP_ADDON_RFP = (ERROR_BASE_SIGFOX_EP_LIB + (SIGFOX_ERROR_SOURCE_LAST * ERROR_BASE_STEP)),
    // Last base value.
    ERROR_BASE_LAST = (ERROR_BASE_SIGFOX_EP_ADDON_RFP + ERROR_BASE_STEP)
} ERROR_base_t;

#endif /* __ERROR_BASE_H__ */
