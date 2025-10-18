/*
 * fxls89xxxx_configuration.c
 *
 *  Created on: 17 oct. 2025
 *      Author: Ludo
 */

#include "fxls89xxxx_configuration.h"

#include "fxls89xxxx.h"
#include "hmd_flags.h"

#ifdef HMD_ACCELEROMETER_ENABLE

/*** FXLS89XXXX CONFIGURATION global variables ***/

const FXLS89XXXX_register_setting_t FXLS89XXXX_ACTIVE_CONFIGURATION[FXLS89XXXX_ACTIVE_CONFIGURATION_SIZE] = {
    { FXLS89XXXX_REGISTER_SENS_CONFIG1, 0x00 }, // // Full scale = +/-2g.
    { FXLS89XXXX_REGISTER_SENS_CONFIG3, 0x99 }, // ODR = 6.25Hz.
    { FXLS89XXXX_REGISTER_SDCD_UTHS_MSB, 0x00 }, // High threshold delta = +0.1g.
    { FXLS89XXXX_REGISTER_SDCD_UTHS_LSB, 0x66 },
    { FXLS89XXXX_REGISTER_SDCD_LTHS_MSB, 0x0F }, // Low threshold delta = -0.1g.
    { FXLS89XXXX_REGISTER_SDCD_LTHS_LSB, 0x9A },
    { FXLS89XXXX_REGISTER_INT_EN, 0x20 }, // Enable outside threshold interrupt.
    { FXLS89XXXX_REGISTER_SENS_CONFIG4, 0x01 }, // INT polarity is active high.
    { FXLS89XXXX_REGISTER_INT_PIN_SEL, 0x00 }, // Use INT1 pin.
    { FXLS89XXXX_REGISTER_SDCD_CONFIG1, 0x38 }, // Enable interrupt on all axis.
    { FXLS89XXXX_REGISTER_SDCD_CONFIG2, 0xD8 }, // Enable SDCD, enable relative mode and disable debouncing.
    { FXLS89XXXX_REGISTER_SENS_CONFIG1, 0x01 } // ACTIVE='1'.
};

const FXLS89XXXX_register_setting_t FXLS89XXXX_SLEEP_CONFIGURATION[FXLS89XXXX_SLEEP_CONFIGURATION_SIZE] = {
    { FXLS89XXXX_REGISTER_SENS_CONFIG1, 0x00 }, // ACTIVE='0'.
};

#endif /* HMD_ACCELEROMETER_ENABLE */
