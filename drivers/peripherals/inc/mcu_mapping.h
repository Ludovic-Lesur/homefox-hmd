/*
 * mcu_mapping.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __MCU_MAPPING_H__
#define __MCU_MAPPING_H__

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"

/*** MCU MAPPING macros ***/

#define ADC_CHANNEL_VBATT       ADC_CHANNEL_IN1

#define I2C_INSTANCE_SENSORS    I2C_INSTANCE_I2C1

#define SPI_INSTANCE_RADIO      SPI_INSTANCE_SPI1

#define TIM_INSTANCE_MCU_API    TIM_INSTANCE_TIM2

#define USART_INSTANCE_AT       USART_INSTANCE_USART1

/*** MCU MAPPING structures ***/

/*!******************************************************************
 * \enum ADC_channel_index_t
 * \brief ADC channels index.
 *******************************************************************/
typedef enum {
    ADC_CHANNEL_INDEX_VBATT_MEASURE = 0,
    ADC_CHANNEL_INDEX_LAST
} ADC_channel_index_t;

/*** MCU MAPPING global variables ***/

// Analog inputs.
extern const GPIO_pin_t GPIO_MNTR_EN;
extern const ADC_gpio_t ADC_GPIO;
// Button.
extern const GPIO_pin_t GPIO_BUTTON;
// Radio power control.
extern const GPIO_pin_t GPIO_RF_POWER_ENABLE;
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
// RF switch.
extern const GPIO_pin_t GPIO_RF_SWITCH_CONTROL;
// SX1261.
extern const SPI_gpio_t SPI_GPIO_SX1261;
extern const GPIO_pin_t GPIO_SX1261_CS;
extern const GPIO_pin_t GPIO_SX1261_NRESET;
extern const GPIO_pin_t GPIO_SX1261_BUSY;
extern const GPIO_pin_t GPIO_SX1261_DIO1;
extern const GPIO_pin_t GPIO_SX1261_DIO2;
// Sensors.
extern const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE;
extern const I2C_gpio_t I2C_GPIO_SENSORS;
extern const GPIO_pin_t GPIO_SENSOR_IRQ;
// AT interface.
extern const USART_gpio_t USART_GPIO_AT;
// RGB LED
extern const GPIO_pin_t GPIO_LED_RED;
extern const GPIO_pin_t GPIO_LED_GREEN;
extern const GPIO_pin_t GPIO_LED_BLUE;

#endif /* __MCU_MAPPING_H__ */
