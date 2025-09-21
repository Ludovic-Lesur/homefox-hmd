/*
 * i2c_address.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __I2C_ADDRESS_H__
#define __I2C_ADDRESS_H__

/*!******************************************************************
 * \enum I2C_address_mapping_t
 * \brief I2C slaves address mapping.
 *******************************************************************/
typedef enum {
    I2C_ADDRESS_SHT30 = 0x44,
    I2C_ADDRESS_ENS160 = 0x53,
    I2C_ADDRESS_SGP40 = 0x59,
    I2C_ADDRESS_FXLS8974CF = 0x18
} I2C_address_mapping_t;

#endif /* __I2C_ADDRESS_H__ */
