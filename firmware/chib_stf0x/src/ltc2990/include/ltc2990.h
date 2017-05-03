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

#define        I2C_WRITEADDR(ADDR) ((ADDR) | 0x1)

#define        LTC2990_I2C_TX_BUFSIZE  4U
#define        LTC2990_I2C_RX_BUFSIZE  20U

extern const uint8_t LTC2990_I2C_ADDR;

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

#ifdef __cplusplus
}
#endif

#endif
//! @}
