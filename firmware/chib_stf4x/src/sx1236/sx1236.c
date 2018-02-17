/*! \file sx1236.c
 *
 * API for sx1236.c
 */

/*!
 * \defgroup sx1236 Semtech Radio Chip
 *
 * @{
 */

/*
 *     Adapted from:
 *
 *     Programmed by William Harrington and Michael Mathis for
 *     the low-gain-radio capstone 2016.
 *     https://github.com/oresat/low-gain-radio.git
 *
 *     Modified by Evan Yand for C3 board 2017.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "sx1236.h"


#define SPI_MAKE_WRITE_ADDR(a) (a|0x80U)

uint8_t       sx_txbuff[MAX_SX_BUFF];
uint8_t       sx_rxbuff[MAX_SX_BUFF];

/* assign the addresses to struct for registers in transceiver block */
struct SX1236 regaddrs =
{
	.RegFifo            = 0x00,
	.RegOpMode          = 0x01,
	.RegBitrateMsb      = 0x02,
	.RegBitrateLsb      = 0x03,
	.RegFdevMsb         = 0x04,
	.RegFdevLsb         = 0x05,
	.RegFrfMsb          = 0x06,
	.RegFrfMid          = 0x07,
	.RegFrfLsb          = 0x08,
	.RegPaConfig        = 0x09,
	.RegPaRamp          = 0x0A,
	.RegOcp             = 0x0B,
	.RegLna             = 0x0C,
	.RegRxConfig        = 0x0D,
	.RegRssiConfig      = 0x0E,
	.RegRssiCollision   = 0x0F,
	.RegRssiThresh      = 0x10,
	.RegRssiValue       = 0x11,
	.RegRxBw            = 0x12,
	.RegAfcBw           = 0x13,
	.RegOokPeak         = 0x14,
	.RegOokFix          = 0x15,
	.RegOokAvg          = 0x16,
	.RegAfcFei          = 0x1A,
	.RegAfcMsb          = 0x1B,
	.RegAfcLsb          = 0x1C,
	.RegFeiMsb          = 0x1D,
	.RegFeiLsb          = 0x1E,
	.RegPreambleDetect  = 0x1F,
	.RegRxTimeout1      = 0x20,
	.RegRxTimeout2      = 0x21,
	.RegRxTimeout3      = 0x22,
	.RegRxDelay         = 0x23,
	.RegOsc             = 0x24,
	.RegPreambleMsb     = 0x25,
	.RegPreambleLsb     = 0x26,
	.RegSyncConfig      = 0x27,
	.RegSyncValue1      = 0x28,
	.RegSyncValue2      = 0x29,
	.RegSyncValue3      = 0x2A,
	.RegSyncValue4      = 0x2B,
	.RegSyncValue5      = 0x2C,
	.RegSyncValue6      = 0x2D,
	.RegSyncValue7      = 0x2E,
	.RegSyncValue8      = 0x2F,
	.RegPacketConfig1   = 0x30,
	.RegPacketConfig2   = 0x31,
	.RegPayloadLength   = 0x32,
	.RegNodeAdrs        = 0x33,
	.RegBroadcastAdrs   = 0x34,
	.RegFifoThresh      = 0x35,
	.RegSeqConfig1      = 0x36,
	.RegSeqConfig2      = 0x37,
	.RegTimerResol      = 0x38,
	.RegTimer1Coef      = 0x39,
	.RegTimer2Coef      = 0x3A,
	.RegImageCal        = 0x3B,
	.RegTemp            = 0x3C,
	.RegLowBat          = 0x3D,
	.RegIrqFlags1       = 0x3E,
	.RegIrqFlags2       = 0x3F,
	.RegDioMapping1     = 0x40,
	.RegDioMapping2     = 0x41,
	.RegVersion         = 0x42,
	.RegPllHop          = 0x44,
	.RegTcxo            = 0x4B,
	.RegPaDac           = 0x4D,
	.RegFormerTemp      = 0x5B,
	.RegBitRateFrac     = 0x5D,
	.RegAgcRef          = 0x61,
	.RegAgcThresh1      = 0x62,
	.RegAgcThresh2      = 0x63,
	.RegAgcThresh3      = 0x64,
	.RegPllLf           = 0x70,
};

/*
 *  Default values set upon POR
 *  These are forced by Semtech
 *
 *  Corrected some of the values to reflect POR values.
 *  G.N. LeBrasseur  1-Sep-2017
 *
 */
struct SX1236 POR_defaults =
{
	.RegFifo            = 0x00,
	.RegOpMode          = 0x09,     // Was incorrect with 0x01
	.RegBitrateMsb      = 0x1A,
	.RegBitrateLsb      = 0x0B,
	.RegFdevMsb         = 0x00,
	.RegFdevLsb         = 0x52,
	.RegFrfMsb          = 0x6C,
	.RegFrfMid          = 0x80,
	.RegFrfLsb          = 0x00,
	.RegPaConfig        = 0x4F,
	.RegPaRamp          = 0x09,
	.RegOcp             = 0x2B,
	.RegLna             = 0x20,
	.RegRxConfig        = 0x0E,     // Was incorrect with 0x08
	.RegRssiConfig      = 0x02,
	.RegRssiCollision   = 0x0A,
	.RegRssiThresh      = 0xFF,
	.RegRssiValue       = 0x00,
	.RegRxBw            = 0x15,
	.RegAfcBw           = 0x0B,
	.RegOokPeak         = 0x28,
	.RegOokFix          = 0x0C,
	.RegOokAvg          = 0x12,
	.Reserved17         = 0x47,
	.Reserved18         = 0x32,
	.Reserved19         = 0x3e,
	.RegAfcFei          = 0x00,
	.RegAfcMsb          = 0x00,
	.RegAfcLsb          = 0x00,
	.RegFeiMsb          = 0x00,
	.RegFeiLsb          = 0x00,
	.RegPreambleDetect  = 0xAA,     // Was incorrect with 0x40
	.RegRxTimeout1      = 0x00,
	.RegRxTimeout2      = 0x00,
	.RegRxTimeout3      = 0x00,
	.RegRxDelay         = 0x00,
	.RegOsc             = 0x07,     // Was incorrct with 0x05
	.RegPreambleMsb     = 0x00,
	.RegPreambleLsb     = 0x03,
	.RegSyncConfig      = 0x93,
	.RegSyncValue1      = 0x01,     // Was incorrect with 0x55
	.RegSyncValue2      = 0x01,     // Was incorrect with 0x55
	.RegSyncValue3      = 0x01,     // Was incorrect with 0x55
	.RegSyncValue4      = 0x01,     // Was incorrect with 0x55
	.RegSyncValue5      = 0x01,     // Was incorrect with 0x55
	.RegSyncValue6      = 0x01,     // Was incorrect with 0x55
	.RegSyncValue7      = 0x01,     // Was incorrect with 0x55
	.RegSyncValue8      = 0x01,     // Was incorrect with 0x55
	.RegPacketConfig1   = 0x90,
	.RegPacketConfig2   = 0x40,
	.RegPayloadLength   = 0x40,
	.RegNodeAdrs        = 0x00,
	.RegBroadcastAdrs   = 0x00,
	.RegFifoThresh      = 0x8F,     // Was incorrect with 0x0F
	.RegSeqConfig1      = 0x00,
	.RegSeqConfig2      = 0x00,
	.RegTimerResol      = 0x00,
	.RegTimer1Coef      = 0xF5,
	.RegTimer2Coef      = 0x20,
	.RegImageCal        = 0x02,     // Was incorrect with 0x82
	.RegTemp            = 0x00,
	.RegLowBat          = 0x02,
	.RegIrqFlags1       = 0x00,     // Was incorrect with 0x80
	.RegIrqFlags2       = 0x00,     // Was incorrect with 0x40 (possiblly in error?)
	.RegDioMapping1     = 0x00,
	.RegDioMapping2     = 0x00,
	.RegVersion         = 0x00,     // Was incorrect with 0x12 (read only value should be 0x12)
	.RegPllHop          = 0x2D,
	.RegTcxo            = 0x09,
	.RegPaDac           = 0x84,
	.RegFormerTemp      = 0x00,
	.RegBitRateFrac     = 0x00,
	//  Band specific additional registers; Two copies depending on RegOpMode.LowFrequencyModeOn; POR default is LF (i.e. 1)
	.RegAgcRef          = 0x19,     // Was incorrect with 0x13
	.RegAgcThresh1      = 0x0C,     // Was incorrect with 0x0E
	.RegAgcThresh2      = 0x4B,     // Was incorrect with 0x5B
	.RegAgcThresh3      = 0xCC,     // Was incorrect with 0xDB
	.RegPllLf           = 0xD0
};

void sx1236_print_regs(SPIDriver * spip)
{
	/* Dumps all registers to serial out */

 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegOpMode", sx1236_read_reg(spip, regaddrs.RegOpMode), POR_defaults.RegOpMode);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegBitrateMsb", sx1236_read_reg(spip, regaddrs.RegBitrateMsb), POR_defaults.RegBitrateMsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegBitrateLsb", sx1236_read_reg(spip, regaddrs.RegBitrateLsb), POR_defaults.RegBitrateLsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFdevMsb", sx1236_read_reg(spip, regaddrs.RegFdevMsb), POR_defaults.RegFdevMsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFdevLsb", sx1236_read_reg(spip, regaddrs.RegFdevLsb), POR_defaults.RegFdevLsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFrfMsb", sx1236_read_reg(spip, regaddrs.RegFrfMsb), POR_defaults.RegFrfMsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFrfMid", sx1236_read_reg(spip, regaddrs.RegFrfMid), POR_defaults.RegFrfMid);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFrfLsb", sx1236_read_reg(spip, regaddrs.RegFrfLsb), POR_defaults.RegFrfLsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPaConfig", sx1236_read_reg(spip, regaddrs.RegPaConfig), POR_defaults.RegPaConfig);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPaRamp", sx1236_read_reg(spip, regaddrs.RegPaRamp), POR_defaults.RegPaRamp);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegOcp", sx1236_read_reg(spip, regaddrs.RegOcp), POR_defaults.RegOcp);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegLna", sx1236_read_reg(spip, regaddrs.RegLna), POR_defaults.RegLna);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRxConfig", sx1236_read_reg(spip, regaddrs.RegRxConfig), POR_defaults.RegRxConfig);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRssiConfig", sx1236_read_reg(spip, regaddrs.RegRssiConfig), POR_defaults.RegRssiConfig);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRssiCollision", sx1236_read_reg(spip, regaddrs.RegRssiCollision), POR_defaults.RegRssiCollision);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRssiThresh", sx1236_read_reg(spip, regaddrs.RegRssiThresh), POR_defaults.RegRssiThresh);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRssiValue", sx1236_read_reg(spip, regaddrs.RegRssiValue), POR_defaults.RegRssiValue);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRxBw", sx1236_read_reg(spip, regaddrs.RegRxBw), POR_defaults.RegRxBw);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAfcBw", sx1236_read_reg(spip, regaddrs.RegAfcBw), POR_defaults.RegAfcBw);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegOokPeak", sx1236_read_reg(spip, regaddrs.RegOokPeak), POR_defaults.RegOokPeak);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegOokFix", sx1236_read_reg(spip, regaddrs.RegOokFix), POR_defaults.RegOokFix);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegOokAvg", sx1236_read_reg(spip, regaddrs.RegOokAvg), POR_defaults.RegOokAvg);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","Reserved17", sx1236_read_reg(spip, regaddrs.Reserved17), POR_defaults.Reserved17);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","Reserved18", sx1236_read_reg(spip, regaddrs.Reserved18), POR_defaults.Reserved18);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","Reserved19", sx1236_read_reg(spip, regaddrs.Reserved19), POR_defaults.Reserved19);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAfcFei", sx1236_read_reg(spip, regaddrs.RegAfcFei), POR_defaults.RegAfcFei);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAfcMsb", sx1236_read_reg(spip, regaddrs.RegAfcMsb), POR_defaults.RegAfcMsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAfcLsb", sx1236_read_reg(spip, regaddrs.RegAfcLsb), POR_defaults.RegAfcLsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFeiMsb", sx1236_read_reg(spip, regaddrs.RegFeiMsb), POR_defaults.RegFeiMsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFeiLsb", sx1236_read_reg(spip, regaddrs.RegFeiLsb), POR_defaults.RegFeiLsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPreambleDetect", sx1236_read_reg(spip, regaddrs.RegPreambleDetect), POR_defaults.RegPreambleDetect);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRxTimeout1", sx1236_read_reg(spip, regaddrs.RegRxTimeout1), POR_defaults.RegRxTimeout1);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRxTimeout2", sx1236_read_reg(spip, regaddrs.RegRxTimeout2), POR_defaults.RegRxTimeout2);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRxTimeout3", sx1236_read_reg(spip, regaddrs.RegRxTimeout3), POR_defaults.RegRxTimeout3);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegRxDelay", sx1236_read_reg(spip, regaddrs.RegRxDelay), POR_defaults.RegRxDelay);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegOsc", sx1236_read_reg(spip, regaddrs.RegOsc), POR_defaults.RegOsc);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPreambleMsb", sx1236_read_reg(spip, regaddrs.RegPreambleMsb), POR_defaults.RegPreambleMsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPreambleLsb", sx1236_read_reg(spip, regaddrs.RegPreambleLsb), POR_defaults.RegPreambleLsb);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncConfig", sx1236_read_reg(spip, regaddrs.RegSyncConfig), POR_defaults.RegSyncConfig);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue1", sx1236_read_reg(spip, regaddrs.RegSyncValue1), POR_defaults.RegSyncValue1);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue2", sx1236_read_reg(spip, regaddrs.RegSyncValue2), POR_defaults.RegSyncValue2);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue3", sx1236_read_reg(spip, regaddrs.RegSyncValue3), POR_defaults.RegSyncValue3);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue4", sx1236_read_reg(spip, regaddrs.RegSyncValue4), POR_defaults.RegSyncValue4);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue5", sx1236_read_reg(spip, regaddrs.RegSyncValue5), POR_defaults.RegSyncValue5);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue6", sx1236_read_reg(spip, regaddrs.RegSyncValue6), POR_defaults.RegSyncValue6);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue7", sx1236_read_reg(spip, regaddrs.RegSyncValue7), POR_defaults.RegSyncValue7);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSyncValue8", sx1236_read_reg(spip, regaddrs.RegSyncValue8), POR_defaults.RegSyncValue8);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPacketConfig1", sx1236_read_reg(spip, regaddrs.RegPacketConfig1), POR_defaults.RegPacketConfig1);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPacketConfig2", sx1236_read_reg(spip, regaddrs.RegPacketConfig2), POR_defaults.RegPacketConfig2);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPayloadLength", sx1236_read_reg(spip, regaddrs.RegPayloadLength), POR_defaults.RegPayloadLength);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegNodeAdrs", sx1236_read_reg(spip, regaddrs.RegNodeAdrs), POR_defaults.RegNodeAdrs);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegBroadcastAdrs", sx1236_read_reg(spip, regaddrs.RegBroadcastAdrs), POR_defaults.RegBroadcastAdrs);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFifoThresh", sx1236_read_reg(spip, regaddrs.RegFifoThresh), POR_defaults.RegFifoThresh);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSeqConfig1", sx1236_read_reg(spip, regaddrs.RegSeqConfig1), POR_defaults.RegSeqConfig1);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegSeqConfig2", sx1236_read_reg(spip, regaddrs.RegSeqConfig2), POR_defaults.RegSeqConfig2);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegTimerResol", sx1236_read_reg(spip, regaddrs.RegTimerResol), POR_defaults.RegTimerResol);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegTimer1Coef", sx1236_read_reg(spip, regaddrs.RegTimer1Coef), POR_defaults.RegTimer1Coef);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegTimer2Coef", sx1236_read_reg(spip, regaddrs.RegTimer2Coef), POR_defaults.RegTimer2Coef);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegImageCal", sx1236_read_reg(spip, regaddrs.RegImageCal), POR_defaults.RegImageCal);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegTemp", sx1236_read_reg(spip, regaddrs.RegTemp), POR_defaults.RegTemp);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegLowBat", sx1236_read_reg(spip, regaddrs.RegLowBat), POR_defaults.RegLowBat);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegIrqFlags1", sx1236_read_reg(spip, regaddrs.RegIrqFlags1), POR_defaults.RegIrqFlags1);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegIrqFlags2", sx1236_read_reg(spip, regaddrs.RegIrqFlags2), POR_defaults.RegIrqFlags2);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegDioMapping1", sx1236_read_reg(spip, regaddrs.RegDioMapping1), POR_defaults.RegDioMapping1);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegDioMapping2", sx1236_read_reg(spip, regaddrs.RegDioMapping2), POR_defaults.RegDioMapping2);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegVersion", sx1236_read_reg(spip, regaddrs.RegVersion), POR_defaults.RegVersion);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPllHop", sx1236_read_reg(spip, regaddrs.RegPllHop), POR_defaults.RegPllHop);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegTcxo", sx1236_read_reg(spip, regaddrs.RegOpMode), POR_defaults.RegOpMode);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPaDac", sx1236_read_reg(spip, regaddrs.RegPaDac), POR_defaults.RegPaDac);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegFormerTemp", sx1236_read_reg(spip, regaddrs.RegFormerTemp), POR_defaults.RegFormerTemp);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegBitRateFrac", sx1236_read_reg(spip, regaddrs.RegBitRateFrac), POR_defaults.RegBitRateFrac);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAgcRef", sx1236_read_reg(spip, regaddrs.RegAgcRef), POR_defaults.RegAgcRef);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAgcThresh1", sx1236_read_reg(spip, regaddrs.RegAgcThresh1), POR_defaults.RegAgcThresh1);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAgcThresh2", sx1236_read_reg(spip, regaddrs.RegAgcThresh2), POR_defaults.RegAgcThresh2);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegAgcThresh3", sx1236_read_reg(spip, regaddrs.RegAgcThresh3), POR_defaults.RegAgcThresh3);
 	chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n","RegPllLf", sx1236_read_reg(spip, regaddrs.RegPllLf), POR_defaults.RegPllLf);


	chprintf(DEBUG_CHP, "\n\n\n");
}


/* sx1236 Manual reset spec
   7.2.2.  Manual Reset
    A manual reset of the SX1236 is possible even for applications in which VDD
    cannot be physically disconnected. Pin 7 should be pulled low for a hundred
    microseconds, and then released. The user should then wait for 5 ms before
    using the chip.
*/
void sx1236_reset(void)
{
	palSetPad(GPIOA, GPIOA_SEMTECH_RST);
	chThdSleepMilliseconds(10);
	palClearPad(GPIOA, GPIOA_SEMTECH_RST);
	chThdSleepMilliseconds(10);
	palSetPad(GPIOA, GPIOA_SEMTECH_RST);
	chThdSleepMilliseconds(10);  // Let chip reset - conservative
}

void sx1236_read(SPIDriver * spip, uint8_t address, uint8_t * rx_buf, uint8_t n)
{
	spiSelect(spip);

	spiStartSend(spip, 1, &address);
	while((*spip).state != SPI_READY) { }

	spiStartReceive(spip, n, rx_buf);
	while((*spip).state != SPI_READY) { }

	spiUnselect(spip);
}

uint8_t sx1236_read_reg(SPIDriver * spip, uint8_t address)
{
	sx1236_read(spip, address, sx_rxbuff, 1);
	return(sx_rxbuff[0]);
}

void sx1236_write(SPIDriver * spip, uint8_t address, uint8_t * tx_buf, uint8_t n)
{
	uint8_t addrSend = SPI_MAKE_WRITE_ADDR(address);

	spiSelect(spip);

	spiStartSend(spip, 1, &addrSend);
	while((*spip).state != SPI_READY) {}

	spiStartSend(spip, n, tx_buf);
	while((*spip).state != SPI_READY) {}

	spiUnselect(spip);
}

void sx1236_write_reg(SPIDriver * spip, uint8_t address, uint8_t newval)
{
	sx_txbuff[0] = newval;
	sx1236_write(spip, address, sx_txbuff, 1);
}


void sx1236_check_reg(SPIDriver * spip, uint8_t address, uint8_t checkval)
{
	sx1236_read(spip, address, sx_rxbuff, 1);
	if(sx_rxbuff[0] != checkval)
	{
		chprintf(DEBUG_CHP, "%s:%d\tReg:\t0x%x\tGot:\t0x%x not 0x%x\r\n",  __FILE__, __LINE__, address,  sx_rxbuff[0], checkval);
	}
	chprintf(DEBUG_CHP, "Reg:\t0x%x set to:\t0x%x\r\n",  address,  sx_rxbuff[0]);
}

void sx1236_write_carrier_freq(SPIDriver * spip, config_sx1236 * c)
{
	uint32_t frf      = 0;

	frf               = (uint32_t)incr_rnd(((1.0 * c->carrier_freq) / c->Fstep ), 1);

	c->sx1236_state.RegFrfMsb      = (frf >> 16) & 0xff;
	c->sx1236_state.RegFrfMid      = (frf >> 8)  & 0xff;
	c->sx1236_state.RegFrfLsb      = frf         & 0xff;

	sx_txbuff[0]                   = c->sx1236_state.RegFrfMsb;
	sx_txbuff[1]                   = c->sx1236_state.RegFrfMid;
	sx_txbuff[2]                   = c->sx1236_state.RegFrfLsb;

	sx1236_write(spip, regaddrs.RegFrfMsb, sx_txbuff, 3);
}

void sx1236_set_freq_deviation(SPIDriver * spip, config_sx1236 * c)
{
	uint32_t    freqdev      = 0;

	freqdev          = (uint32_t)incr_rnd((1.0 * c->freq_dev_hz / c->Fstep), 1);

	chprintf(DEBUG_CHP, "freqdev: 0x%x\t%d\r\n", freqdev, freqdev);
	c->sx1236_state.RegFdevMsb      = (freqdev >> 8) & 0x3f;
	c->sx1236_state.RegFdevLsb      = freqdev        & 0xff;

	sx_txbuff[0] = c->sx1236_state.RegFdevMsb;
	sx_txbuff[1] = c->sx1236_state.RegFdevLsb;

	sx1236_write(    spip, regaddrs.RegFdevMsb, sx_txbuff,   2);
	sx1236_check_reg(spip, regaddrs.RegFdevMsb, c->sx1236_state.RegFdevMsb);
	sx1236_check_reg(spip, regaddrs.RegFdevLsb,  c->sx1236_state.RegFdevLsb);
}

void sx1236_set_bitrate(SPIDriver * spip, config_sx1236 * c)
{
	uint32_t    rate         = 0;

	rate             = (uint32_t)incr_rnd(((1.0 * c->Fxosc) / c->bitrate), 1);

	c->sx1236_state.RegBitrateMsb = (rate >> 8) & 0xff;
	c->sx1236_state.RegBitrateLsb = rate        & 0xff;

	sx_txbuff[0]     = c->sx1236_state.RegBitrateMsb;
	sx_txbuff[1]     = c->sx1236_state.RegBitrateLsb;

	sx1236_write(spip, regaddrs.RegBitrateMsb, sx_txbuff, 2);

	//sx1236_check_reg(spip, regaddrs.RegBitrateMsb, c->sx1236_state.RegBitrateMsb);
	//sx1236_check_reg(spip, regaddrs.RegBitrateLsb, c->sx1236_state.RegBitrateLsb);
}

/*
 * Set an SX1236 struct to POR state
 * (This is just a struct deep copy.)
 */
void sx1236_init_state(struct SX1236 * s)
{
	s->RegFifo            = POR_defaults.RegFifo;
	s->RegOpMode          = POR_defaults.RegOpMode;
	s->RegBitrateMsb      = POR_defaults.RegBitrateMsb;
	s->RegBitrateLsb      = POR_defaults.RegBitrateLsb;
	s->RegFdevMsb         = POR_defaults.RegFdevMsb;
	s->RegFdevLsb         = POR_defaults.RegFdevLsb;
	s->RegFrfMsb          = POR_defaults.RegFrfMsb;
	s->RegFrfMid          = POR_defaults.RegFrfMid;
	s->RegFrfLsb          = POR_defaults.RegFrfLsb;
	s->RegPaConfig        = POR_defaults.RegPaConfig;
	s->RegPaRamp          = POR_defaults.RegPaRamp;
	s->RegOcp             = POR_defaults.RegOcp;
	s->RegLna             = POR_defaults.RegLna;
	s->RegRxConfig        = POR_defaults.RegRxConfig;
	s->RegRssiConfig      = POR_defaults.RegRssiConfig;
	s->RegRssiCollision   = POR_defaults.RegRssiCollision;
	s->RegRssiThresh      = POR_defaults.RegRssiThresh;
	s->RegRssiValue       = POR_defaults.RegRssiValue;
	s->RegRxBw            = POR_defaults.RegRxBw;
	s->RegAfcBw           = POR_defaults.RegAfcBw;
	s->RegOokPeak         = POR_defaults.RegOokPeak;
	s->RegOokFix          = POR_defaults.RegOokFix;
	s->RegOokAvg          = POR_defaults.RegOokAvg;
	s->RegAfcFei          = POR_defaults.RegAfcFei;
	s->RegAfcMsb          = POR_defaults.RegAfcMsb;
	s->RegAfcLsb          = POR_defaults.RegAfcLsb;
	s->RegFeiMsb          = POR_defaults.RegFeiMsb;
	s->RegFeiLsb          = POR_defaults.RegFeiLsb;
	s->RegPreambleDetect  = POR_defaults.RegPreambleDetect;
	s->RegRxTimeout1      = POR_defaults.RegRxTimeout1;
	s->RegRxTimeout2      = POR_defaults.RegRxTimeout2;
	s->RegRxTimeout3      = POR_defaults.RegRxTimeout3;
	s->RegRxDelay         = POR_defaults.RegRxDelay;
	s->RegOsc             = POR_defaults.RegOsc;
	s->RegPreambleMsb     = POR_defaults.RegPreambleMsb;
	s->RegPreambleLsb     = POR_defaults.RegPreambleLsb;
	s->RegSyncConfig      = POR_defaults.RegSyncConfig;
	s->RegSyncValue1      = POR_defaults.RegSyncValue1;
	s->RegSyncValue2      = POR_defaults.RegSyncValue2;
	s->RegSyncValue3      = POR_defaults.RegSyncValue3;
	s->RegSyncValue4      = POR_defaults.RegSyncValue4;
	s->RegSyncValue5      = POR_defaults.RegSyncValue5;
	s->RegSyncValue6      = POR_defaults.RegSyncValue6;
	s->RegSyncValue7      = POR_defaults.RegSyncValue7;
	s->RegSyncValue8      = POR_defaults.RegSyncValue8;
	s->RegPacketConfig1   = POR_defaults.RegPacketConfig1;
	s->RegPacketConfig2   = POR_defaults.RegPacketConfig2;
	s->RegPayloadLength   = POR_defaults.RegPayloadLength;
	s->RegNodeAdrs        = POR_defaults.RegNodeAdrs;
	s->RegBroadcastAdrs   = POR_defaults.RegBroadcastAdrs;
	s->RegFifoThresh      = POR_defaults.RegFifoThresh;
	s->RegSeqConfig1      = POR_defaults.RegSeqConfig1;
	s->RegSeqConfig2      = POR_defaults.RegSeqConfig2;
	s->RegTimerResol      = POR_defaults.RegTimerResol;
	s->RegTimer1Coef      = POR_defaults.RegTimer1Coef;
	s->RegTimer2Coef      = POR_defaults.RegTimer2Coef;
	s->RegImageCal        = POR_defaults.RegImageCal;
	s->RegTemp            = POR_defaults.RegTemp;
	s->RegLowBat          = POR_defaults.RegLowBat;
	s->RegIrqFlags1       = POR_defaults.RegIrqFlags1;
	s->RegIrqFlags2       = POR_defaults.RegIrqFlags2;
	s->RegDioMapping1     = POR_defaults.RegDioMapping1;
	s->RegDioMapping2     = POR_defaults.RegDioMapping2;
	s->RegVersion         = POR_defaults.RegVersion;
	s->RegPllHop          = POR_defaults.RegPllHop;
	s->RegTcxo            = POR_defaults.RegTcxo;
	s->RegPaDac           = POR_defaults.RegPaDac;
	s->RegFormerTemp      = POR_defaults.RegFormerTemp;
	s->RegBitRateFrac     = POR_defaults.RegBitRateFrac;
	s->RegAgcRef          = POR_defaults.RegAgcRef;
	s->RegAgcThresh1      = POR_defaults.RegAgcThresh1;
	s->RegAgcThresh2      = POR_defaults.RegAgcThresh2;
	s->RegAgcThresh3      = POR_defaults.RegAgcThresh3;
	s->RegPllLf           = POR_defaults.RegPllLf;
};

/*
 * write FIFO
 */
void sx1236_write_FIFO(SPIDriver * spip, uint8_t value)
{
	while ( palReadPad(GPIOC, GPIOC_SX_DIO2)){
	  		chprintf(DEBUG_CHP, "FIFO Full\r\n");
			chThdSleepMilliseconds(1);
	}	
	sx1236_write_reg(spip, regaddrs.RegFifo,  value         );
	chprintf(DEBUG_CHP, "\r\r-- 0x%x --\r\n", value);
}


/*
 * Read FIFO
 */
uint8_t sx1236_read_FIFO(SPIDriver * spip)
{
	uint8_t value;
	value = sx1236_read_reg(spip, regaddrs.RegFifo);
	return value;
}



/*
 * Configure to a state given in a config_sx1236 structure
 *
 * Could have multiple state setups for tx, rx, tx_packet, rx_packet etc...
 */
void sx1236_configure(SPIDriver * spip, config_sx1236 * c)
{
	
	sx1236_write_carrier_freq(spip, c);
	sx1236_set_freq_deviation(spip, c);
	sx1236_set_bitrate(spip, c);

	//sx1236_write_reg(spip, regaddrs.RegFifo,  c->sx1236_state.RegFifo          );
	sx1236_write_reg(spip, regaddrs.RegOpMode,  c->sx1236_state.RegOpMode        );
	sx1236_write_reg(spip, regaddrs.RegBitrateMsb,  c->sx1236_state.RegBitrateMsb    );
	sx1236_write_reg(spip, regaddrs.RegBitrateLsb,  c->sx1236_state.RegBitrateLsb    );
	sx1236_write_reg(spip, regaddrs.RegFdevMsb,  c->sx1236_state.RegFdevMsb       );
	sx1236_write_reg(spip, regaddrs.RegFdevLsb,  c->sx1236_state.RegFdevLsb       );
	sx1236_write_reg(spip, regaddrs.RegFrfMsb,  c->sx1236_state.RegFrfMsb        );
	sx1236_write_reg(spip, regaddrs.RegFrfMid,  c->sx1236_state.RegFrfMid        );
	sx1236_write_reg(spip, regaddrs.RegFrfLsb,  c->sx1236_state.RegFrfLsb        );
	sx1236_write_reg(spip, regaddrs.RegPaConfig,  c->sx1236_state.RegPaConfig      );
	sx1236_write_reg(spip, regaddrs.RegPaRamp,  c->sx1236_state.RegPaRamp        );
	sx1236_write_reg(spip, regaddrs.RegOcp,  c->sx1236_state.RegOcp           );
	sx1236_write_reg(spip, regaddrs.RegLna,  c->sx1236_state.RegLna           );
	sx1236_write_reg(spip, regaddrs.RegRxConfig,  c->sx1236_state.RegRxConfig      );
	sx1236_write_reg(spip, regaddrs.RegRssiConfig,  c->sx1236_state.RegRssiConfig    );
	sx1236_write_reg(spip, regaddrs.RegRssiCollision,  c->sx1236_state.RegRssiCollision );
	sx1236_write_reg(spip, regaddrs.RegRssiThresh,  c->sx1236_state.RegRssiThresh    );
	sx1236_write_reg(spip, regaddrs.RegRssiValue,  c->sx1236_state.RegRssiValue     );
	sx1236_write_reg(spip, regaddrs.RegRxBw,  c->sx1236_state.RegRxBw          );
	sx1236_write_reg(spip, regaddrs.RegAfcBw,  c->sx1236_state.RegAfcBw         );
	sx1236_write_reg(spip, regaddrs.RegOokPeak,  c->sx1236_state.RegOokPeak       );
	//sx1236_check_reg(spip, regaddrs.RegOokPeak, c->sx1236_state.RegOokPeak);
	sx1236_write_reg(spip, regaddrs.RegOokFix,  c->sx1236_state.RegOokFix        );
	sx1236_write_reg(spip, regaddrs.RegOokAvg,  c->sx1236_state.RegOokAvg        );
	sx1236_write_reg(spip, regaddrs.RegAfcFei,  c->sx1236_state.RegAfcFei        );
	sx1236_write_reg(spip, regaddrs.RegAfcMsb,  c->sx1236_state.RegAfcMsb        );
	sx1236_write_reg(spip, regaddrs.RegAfcLsb,  c->sx1236_state.RegAfcLsb        );
	sx1236_write_reg(spip, regaddrs.RegFeiMsb,  c->sx1236_state.RegFeiMsb        );
	sx1236_write_reg(spip, regaddrs.RegFeiLsb,  c->sx1236_state.RegFeiLsb        );
	sx1236_write_reg(spip, regaddrs.RegPreambleDetect,  c->sx1236_state.RegPreambleDetect);
	sx1236_write_reg(spip, regaddrs.RegRxTimeout1,  c->sx1236_state.RegRxTimeout1    );
	sx1236_write_reg(spip, regaddrs.RegRxTimeout2,  c->sx1236_state.RegRxTimeout2    );
	sx1236_write_reg(spip, regaddrs.RegRxTimeout3,  c->sx1236_state.RegRxTimeout3    );
	sx1236_write_reg(spip, regaddrs.RegRxDelay,  c->sx1236_state.RegRxDelay       );
	sx1236_write_reg(spip, regaddrs.RegOsc,  c->sx1236_state.RegOsc           );
	sx1236_write_reg(spip, regaddrs.RegPreambleMsb,  c->sx1236_state.RegPreambleMsb   );
	sx1236_write_reg(spip, regaddrs.RegPreambleLsb,  c->sx1236_state.RegPreambleLsb   );
	sx1236_write_reg(spip, regaddrs.RegSyncConfig,  c->sx1236_state.RegSyncConfig    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue1,  c->sx1236_state.RegSyncValue1    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue2,  c->sx1236_state.RegSyncValue2    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue3,  c->sx1236_state.RegSyncValue3    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue4,  c->sx1236_state.RegSyncValue4    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue5,  c->sx1236_state.RegSyncValue5    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue6,  c->sx1236_state.RegSyncValue6    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue7,  c->sx1236_state.RegSyncValue7    );
	sx1236_write_reg(spip, regaddrs.RegSyncValue8,  c->sx1236_state.RegSyncValue8    );
	sx1236_write_reg(spip, regaddrs.RegPacketConfig1,  c->sx1236_state.RegPacketConfig1 );
	sx1236_write_reg(spip, regaddrs.RegPacketConfig2,  c->sx1236_state.RegPacketConfig2 );
	sx1236_write_reg(spip, regaddrs.RegPayloadLength,  c->sx1236_state.RegPayloadLength );
	sx1236_write_reg(spip, regaddrs.RegNodeAdrs,  c->sx1236_state.RegNodeAdrs      );
	sx1236_write_reg(spip, regaddrs.RegBroadcastAdrs,  c->sx1236_state.RegBroadcastAdrs );
	sx1236_write_reg(spip, regaddrs.RegFifoThresh,  c->sx1236_state.RegFifoThresh    );
	sx1236_write_reg(spip, regaddrs.RegSeqConfig1,  c->sx1236_state.RegSeqConfig1    );
	sx1236_write_reg(spip, regaddrs.RegSeqConfig2,  c->sx1236_state.RegSeqConfig2    );
	sx1236_write_reg(spip, regaddrs.RegTimerResol,  c->sx1236_state.RegTimerResol    );
	sx1236_write_reg(spip, regaddrs.RegTimer1Coef,  c->sx1236_state.RegTimer1Coef    );
	sx1236_write_reg(spip, regaddrs.RegTimer2Coef,  c->sx1236_state.RegTimer2Coef    );
	sx1236_write_reg(spip, regaddrs.RegImageCal,  c->sx1236_state.RegImageCal      );
	sx1236_write_reg(spip, regaddrs.RegTemp,  c->sx1236_state.RegTemp          );
	sx1236_write_reg(spip, regaddrs.RegLowBat,  c->sx1236_state.RegLowBat        );
	sx1236_write_reg(spip, regaddrs.RegIrqFlags1,  c->sx1236_state.RegIrqFlags1     );
	sx1236_write_reg(spip, regaddrs.RegIrqFlags2,  c->sx1236_state.RegIrqFlags2     );
	sx1236_write_reg(spip, regaddrs.RegDioMapping1,  c->sx1236_state.RegDioMapping1   );
	sx1236_write_reg(spip, regaddrs.RegDioMapping2,  c->sx1236_state.RegDioMapping2   );
	sx1236_write_reg(spip, regaddrs.RegVersion,  c->sx1236_state.RegVersion       );
	sx1236_write_reg(spip, regaddrs.RegPllHop,  c->sx1236_state.RegPllHop        );
	sx1236_write_reg(spip, regaddrs.RegTcxo,  c->sx1236_state.RegTcxo          );
	sx1236_write_reg(spip, regaddrs.RegPaDac,  c->sx1236_state.RegPaDac         );
	sx1236_write_reg(spip, regaddrs.RegFormerTemp,  c->sx1236_state.RegFormerTemp    );
	sx1236_write_reg(spip, regaddrs.RegBitRateFrac,  c->sx1236_state.RegBitRateFrac   );
	sx1236_write_reg(spip, regaddrs.RegAgcRef,  c->sx1236_state.RegAgcRef        );
	sx1236_write_reg(spip, regaddrs.RegAgcThresh1,  c->sx1236_state.RegAgcThresh1    );
	sx1236_write_reg(spip, regaddrs.RegAgcThresh2,  c->sx1236_state.RegAgcThresh2    );
	sx1236_write_reg(spip, regaddrs.RegAgcThresh3,  c->sx1236_state.RegAgcThresh3    );
	sx1236_write_reg(spip, regaddrs.RegPllLf,  c->sx1236_state.RegPllLf         );

	//sx1236_check_reg(spip, regaddrs.RegOpMode, c->sx1236_state.RegOpMode);
	// Development only: Check register writes
	// sx1236_check_reg(spip, regaddrs.RegOpMode,          c->sx1236_state.RegOpMode);

	//set to receive data from DIO2	
	//palSetPadMode(GPIOC, 2, PAL_MODE_INPUT );
}

void sx1236_packet_tx(SPIDriver * spip, sx1236_packet p)
{
	
	sx1236_write_FIFO(spip, p.PacType );
	sx1236_write_FIFO(spip, p.PacSequence );
	sx1236_write_FIFO(spip, p.PacSourceAddress );
	sx1236_write_FIFO(spip, p.PacDestAddress );
	sx1236_write_FIFO(spip, p.PacInstruction );
	
	for (int i=0; i<PacketContentSize; i++){
		sx1236_write_FIFO(spip, p.PacData[i] );
		//chprintf(DEBUG_CHP, "\r\r-- 0x%x --\r\n", p.PacData[i]);
	}
	//sx1236_write_FIFO(spip, p.PacData[27] );
	//sx1236_write_FIFO(spip, p.PacData[27] );
	//sx1236_write_FIFO(spip, p.PacData[27] );
}

void sx1236_packet_rx(SPIDriver * spip, config_sx1236 * c, sx1236_raw_packet * r)
{
	
	for (int i=0; i<c->sx1236_state.RegPayloadLength; ){
		while ( !palReadPad(GPIOC, GPIOC_SX_DIO3)){			//fifo not empty
			r->RawPacData[i] = sx1236_read_FIFO(spip);
			chprintf(DEBUG_CHP, "\r\r## 0x%x ##\r\n", r->RawPacData[i]);
			i++;
		}
	}
	
}

void sx1236_packet_rx2(SPIDriver * spip, config_sx1236 * c, sx1236_raw_packet * r)
{
	
	for (int i=0; i<c->sx1236_state.RegPayloadLength; ){
		if ( !palReadPad(GPIOC, GPIOC_SX2_DIO3)){			//fifo not empty
			r->RawPacData[i] = sx1236_read_FIFO(spip);
			chprintf(DEBUG_CHP, "\r\r## 0x%x ##\r\n", r->RawPacData[i]);
			i++;
			chThdSleepMilliseconds(5);
		}
	}
	
}

void sx1236_packet_format(sx1236_packet * p, sx1236_raw_packet * r)
{
	
	p->PacType=r->RawPacData[0];
	p->PacSequence=r->RawPacData[1];
	p->PacSourceAddress=r->RawPacData[2];
	p->PacDestAddress=r->RawPacData[3];
	p->PacInstruction=r->RawPacData[4];
	for (int i=0; i<PacketContentSize; i++){
		p->PacData[i] = r->RawPacData[i+5];
	}
}

void sx1236_create_data_packet_tx(SPIDriver * spip, uint8_t data[], int data_size)
{
	
	static uint8_t packetSequence=0;
	sx1236_packet p;
	
	p.PacType=DataPacket;
	p.PacSequence=packetSequence;
	p.PacSourceAddress=Oresat1;
	p.PacDestAddress=BoradcastAddress;
	p.PacInstruction=0x00;

	if (data_size > PacketContentSize){
		chprintf(DEBUG_CHP, "Incorrect Packet\r\n");
	}
	else{
		for (int i=0; i<PacketContentSize; i++){
			if (i <	data_size)	{
				p.PacData[i]=data[i];
				//chprintf(DEBUG_CHP, "\r\n.. 0x%x ..", data[i]);
			}
			else{
				p.PacData[i]=0x00;
			}
			
		}
	}
	
	sx1236_packet_tx(spip, p);
	//chprintf(DEBUG_CHP, "\r\n 2 packet sequence 0x%x ..\r\n", packetSequence);
	packetSequence = packetSequence + 1;
	//chprintf(DEBUG_CHP, "\r\n 3 packet sequence 0x%x ..\r\n", packetSequence);
}

void sx1236_create_instruction_packet_tx(SPIDriver * spip, uint8_t inst)
{
	
	static uint8_t packetSequence=0;
	sx1236_packet p;
	
	p.PacType=InstructionPacket;
	p.PacSequence=packetSequence;
	p.PacSourceAddress=Oresat1;
	p.PacDestAddress=BoradcastAddress;
	p.PacInstruction=inst;

	for (int i=0; i<PacketContentSize; i++){
		p.PacData[i]=0x00;
		p.PacData[i]=i;		//needs to be removed later
	}
	
	sx1236_packet_tx(spip, p);
	chprintf(DEBUG_CHP, "\r\n 2 packet sequence 0x%x ..\r\n", packetSequence);
	packetSequence = packetSequence + 1;
	//chprintf(DEBUG_CHP, "\r\n 3 packet sequence 0x%x ..\r\n", packetSequence);
}

// ! @}

