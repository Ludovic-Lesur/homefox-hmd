/*
 * main.c
 *
 *  Created on: 13 sep. 2025
 *      Author: Ludo
 */

#include "pwr.h"

/*** MAIN functions ***/

/*******************************************************************/
int main(void) {
    // Main loop.
    PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STANDBY);
    return 0;
}
