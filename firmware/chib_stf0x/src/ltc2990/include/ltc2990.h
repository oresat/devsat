/*! \file ltc2990.h */

/*!
 * \addtogroup ltc2990
 *  * @{
 *
 */
#ifndef _LTC2990_H_
#define _LTC2990_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ch.h"
#include "hal.h"

extern const uint8_t LTC2990_I2C_ADDR;

typedef struct ltc2990data
{
	uint8_t STATUS;   // 0x0
	uint8_t CONTROL;  // 0x1
	uint8_t TRIGGER;  // ...
	uint8_t NA;
	uint8_t T_INT_MSB;
	uint8_t T_INT_LSB;
	uint8_t V1_MSB;
	uint8_t V1_LSB;
	uint8_t V2_MSB;
	uint8_t V2_LSB;
	uint8_t V3_MSB;
	uint8_t V3_LSB;
	uint8_t V4_MSB;
	uint8_t V4_LSB;  // ...
	uint8_t VCC_MSB; // 0xe
	uint8_t VCC_LSB; // 0xf
} ltc2990_data;

/* HSI Clock selected 8Mhz - see mcuconf.h Wed 03 May 2017 12:54:49 (PDT) */
/* Ref: 26.4.10 table in Reference Manual for stm32f0 */
typedef enum
{
	I2C_10KHZ_TIMINGR  = 0x1042C3C7,
	I2C_400KHZ_TIMINGR = 0x00310309,
} solar_i2c_cfg;

#define        LTC2990_I2C_TX_BUFSIZE  4U
#define        LTC2990_I2C_RX_BUFSIZE  16U

#define LTC2990_STATUS      0x00U   //  R       Indicates BUSY State, Conversion Sta
#define LTC2990_CONTROL     0x01U   //  R/W     Controls Mode, Single/Repeat, Celsiu
#define LTC2990_TRIGGER     0x02U   //  R/W     Triggers a Conversion
// #define  LTC2990_N/A         0x03    //          Unused Address
#define LTC2990_TINT_MSB    0x04U   //  R       Internal Temperature MSB
#define LTC2990_TINT_LSB    0x05U   //  R       Internal Temperature LSB
#define LTC2990_V1_MSB      0x06U   //  R       V1, V1 – V2 or TR1 MSB
#define LTC2990_V1_LSB      0x07U   //  R       V1, V1 – V2 or TR1 LSB
#define LTC2990_V2_MSB      0x08U   //  R       V2, V1 – V2 or TR1 MSB
#define LTC2990_V2_LSB      0x09U   //  R       V2, V1 – V2 or TR1 LSB
#define LTC2990_V3_MSB      0x0AU   //  R       V3, V3 – V4 or TR2 MSB
#define LTC2990_V3_LSB      0x0BU   //  R       V3, V3 – V4 or TR2 LSB
#define LTC2990_V4_MSB      0x0CU   //  R       V4, V3 – V4 or TR2 MSB
#define LTC2990_V4_LSB      0x0DU   //  R       V4, V3 – V4 or TR2 LSB
#define LTC2990_VCC_MSB     0x0EU   //  R       VCC MSB
#define LTC2990_VCC_LSB     0x0FU   //  R       VCC LSB

#define LTC2990_CONTROL_T_FORMAT_KELVIN     (0b1<<7)
#define LTC2990_CONTROL_ACQ_SINGLE          (0b1<<6)
#define LTC2990_CONTROL_ALL_MODE_4_3        (0b11<<3)
#define LTC2990_CONTROL_MODE_1_2_0          (0b001<<0)

uint8_t ltc2990_readreg(uint8_t reg, i2cflags_t * i2c_errors);
void    ltc2990_writereg(uint8_t reg, uint8_t val, i2cflags_t * i2c_errors);
void ltc2990_read_all(ltc2990_data * d, i2cflags_t * i2c_errors);

#ifdef __cplusplus
}
#endif

#endif
//! @}
