/*
 * gpio_mapping.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "mcu_mapping.h"

#include "gpio.h"
#include "gpio_registers.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"

/*** GPIO MAPPING local global variables ***/

// Analog inputs.
static const GPIO_pin_t GPIO_ADC_VBATT_MEASURE = { GPIOA, 0, 1, 0 };
// Analog inputs list.
static const GPIO_pin_t* const GPIO_ADC_PINS_LIST[ADC_CHANNEL_INDEX_LAST] = { &GPIO_ADC_VBATT_MEASURE };
// I2C1.
static const GPIO_pin_t GPIO_I2C1_SCL = { GPIOB, 1, 6, 1 };
static const GPIO_pin_t GPIO_I2C1_SDA = { GPIOB, 1, 7, 1 };
// SPI1.
static const GPIO_pin_t GPIO_SPI1_SCK = { GPIOA, 0, 5, 0 };
static const GPIO_pin_t GPIO_SPI1_MISO = { GPIOA, 0, 6, 0 };
static const GPIO_pin_t GPIO_SPI1_MOSI = { GPIOA, 0, 7, 0 };
// USART2.
static const GPIO_pin_t GPIO_USART1_TX = { GPIOA, 0, 9, 4 };
static const GPIO_pin_t GPIO_USART1_RX = { GPIOA, 0, 10, 4 };

/*** GPIO MAPPING global variables ***/

// Analog inputs.
const GPIO_pin_t GPIO_MNTR_EN = { GPIOA, 0, 15, 0 };
const ADC_gpio_t ADC_GPIO = { (const GPIO_pin_t**) &GPIO_ADC_PINS_LIST, ADC_CHANNEL_INDEX_LAST };
// Button.
const GPIO_pin_t GPIO_BUTTON = { GPIOA, 0, 2, 0 };
// Radio power control.
const GPIO_pin_t GPIO_RF_POWER_ENABLE = { GPIOA, 0, 12, 0 };
const GPIO_pin_t GPIO_TCXO_POWER_ENABLE = { GPIOB, 1, 2, 0 };
// RF switch.
const GPIO_pin_t GPIO_RF_SWITCH_CONTROL = { GPIOA, 0, 3, 0 };
// S2LP.
const SPI_gpio_t SPI_GPIO_SX1261 = { &GPIO_SPI1_SCK, &GPIO_SPI1_MOSI, &GPIO_SPI1_MISO };
const GPIO_pin_t GPIO_SX1261_CS = { GPIOA, 0, 4, 0 };
const GPIO_pin_t GPIO_SX1261_NRESET = { GPIOB, 1, 0, 0 };
const GPIO_pin_t GPIO_SX1261_BUSY = { GPIOB, 1, 1, 0 };
const GPIO_pin_t GPIO_SX1261_DIO1 = { GPIOA, 0, 8, 0 };
const GPIO_pin_t GPIO_SX1261_DIO2 = { GPIOA, 0, 11, 0 };
// Sensors.
const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE = { GPIOB, 1, 3, 0 };
const I2C_gpio_t I2C_GPIO_SENSORS = { &GPIO_I2C1_SCL, &GPIO_I2C1_SDA };
const GPIO_pin_t GPIO_SENSOR_IRQ = { GPIOA, 0, 0, 0 };
// AT interface.
const USART_gpio_t USART_GPIO_AT = { &GPIO_USART1_TX, &GPIO_USART1_RX };
// RGB LED
const GPIO_pin_t GPIO_LED_RED = { GPIOB, 1, 4, 0 };
const GPIO_pin_t GPIO_LED_GREEN = { GPIOB, 1, 8, 0 };
const GPIO_pin_t GPIO_LED_BLUE = { GPIOB, 1, 5, 0 };
