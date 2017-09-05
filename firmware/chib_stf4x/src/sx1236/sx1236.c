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
	.RegPll             = 0x70,

};

struct SX1236 regdefaults =
{
	.RegFifo            = 0x00,
	.RegOpMode          = 0x01,
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
	.RegRxConfig        = 0x08,
	.RegRssiConfig      = 0x02,
	.RegRssiCollision   = 0x0A,
	.RegRssiThresh      = 0xFF,
	.RegRssiValue       = 0x00,
	.RegRxBw            = 0x15,
	.RegAfcBw           = 0x0B,
	.RegOokPeak         = 0x28,
	.RegOokFix          = 0x0C,
	.RegOokAvg          = 0x12,
	.RegAfcFei          = 0x00,
	.RegAfcMsb          = 0x00,
	.RegAfcLsb          = 0x00,
	.RegFeiMsb          = 0x00,
	.RegFeiLsb          = 0x00,
	.RegPreambleDetect  = 0x40,
	.RegRxTimeout1      = 0x00,
	.RegRxTimeout2      = 0x00,
	.RegRxTimeout3      = 0x00,
	.RegRxDelay         = 0x00,
	.RegOsc             = 0x05,
	.RegPreambleMsb     = 0x00,
	.RegPreambleLsb     = 0x03,
	.RegSyncConfig      = 0x93,
	.RegSyncValue1      = 0x55,
	.RegSyncValue2      = 0x55,
	.RegSyncValue3      = 0x55,
	.RegSyncValue4      = 0x55,
	.RegSyncValue5      = 0x55,
	.RegSyncValue6      = 0x55,
	.RegSyncValue7      = 0x55,
	.RegSyncValue8      = 0x55,
	.RegPacketConfig1   = 0x90,
	.RegPacketConfig2   = 0x40,
	.RegPayloadLength   = 0x40,
	.RegNodeAdrs        = 0x00,
	.RegBroadcastAdrs   = 0x00,
	.RegFifoThresh      = 0x0F,
	.RegSeqConfig1      = 0x00,
	.RegSeqConfig2      = 0x00,
	.RegTimerResol      = 0x00,
	.RegTimer1Coef      = 0xF5,
	.RegTimer2Coef      = 0x20,
	.RegImageCal        = 0x82,
	.RegTemp            = 0x00,
	.RegLowBat          = 0x02,
	.RegIrqFlags1       = 0x80,
	.RegIrqFlags2       = 0x40,
	.RegDioMapping1     = 0x00,
	.RegDioMapping2     = 0x00,
	.RegVersion         = 0x12,
	.RegPllHop          = 0x2D,
	.RegTcxo            = 0x09,
	.RegPaDac           = 0x84,
	.RegFormerTemp      = 0x00,
	.RegBitRateFrac     = 0x00,
	.RegAgcRef          = 0x13,
	.RegAgcThresh1      = 0x0E,
	.RegAgcThresh2      = 0x5B,
	.RegAgcThresh3      = 0xDB,
	.RegPll             = 0xD0,

};

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
	chThdSleepMicroseconds(200);
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


#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)
void sx1236_check_reg(SPIDriver * spip, uint8_t address, uint8_t checkval)
{
	sx1236_read(spip, address, sx_rxbuff, 1);
	if(sx_rxbuff[0] != checkval)
	{
		chprintf(DEBUG_CHP, "%s:%d\tReg:\t0x%x\tGot:\t0x%x not 0x%x\r\n",  __FILE__, __LINE__, address,  sx_rxbuff[0], checkval);
	}
}

void sx1236_write_carrier_freq(SPIDriver * spip, uint32_t carrier_hz, double fstep)
{
	uint32_t frf      = 0.0;
	uint8_t  frf_msb  = 0;
	uint8_t  frf_mid  = 0;
	uint8_t  frf_lsb  = 0;

	frf          = (uint32_t)incr_rnd((carrier_hz / fstep), 0.1);

	frf_msb      = (frf >> 16) & 0xff;
	frf_mid      = (frf >> 8)  & 0xff;
	frf_lsb      = frf       & 0xff;

	sx_txbuff[0] = frf_msb;
	sx_txbuff[1] = frf_mid;
	sx_txbuff[2] = frf_lsb;

	sx1236_write(spip, regaddrs.RegFrfMsb, sx_txbuff, 3);
}

void sx1236_set_freq_deviation(SPIDriver * spip, uint32_t freq_dev_hz, double fstep )
{
	uint32_t    freqdev      = 0;

	uint8_t     freqdev_msb  = 0;
	uint8_t     freqdev_lsb  = 0;

	freqdev          = (uint32_t)incr_rnd((freq_dev_hz / fstep), 0.1);

	freqdev_msb      = (freqdev >> 8) & 0x3f;
	freqdev_lsb      = freqdev      & 0xff;

	sx_txbuff[0] = freqdev_msb;
	sx_txbuff[1] = freqdev_lsb;

	sx1236_write(spip, regaddrs.RegFdevMsb, sx_txbuff, 2);
	// sx1236_check_reg(spip, regaddrs.RegFdevLsb, freqdev_lsb);
	// sx1236_check_reg(spip, regaddrs.RegFdevMsb, freqdev_msb);
}

void sx1236_set_bitrate(SPIDriver * spip, uint32_t fxosc, uint32_t bitrate )
{
	uint32_t    rate         = 0;

	uint8_t     bitrate_msb  = 0;
	uint8_t     bitrate_lsb  = 0;

	rate             = (uint32_t)incr_rnd((fxosc / bitrate), 0.1);

	bitrate_msb      = (rate >> 8) & 0x3f;
	bitrate_lsb      = rate        & 0xff;

	sx_txbuff[0]     = bitrate_msb;
	sx_txbuff[1]     = bitrate_lsb;

	sx1236_write(spip, regaddrs.RegBitrateMsb, sx_txbuff, 2);
	// sx1236_check_reg(spip, regaddrs.RegBitrateLsb, bitrate_lsb);
	// sx1236_check_reg(spip, regaddrs.RegBitrateMsb, bitrate_msb);
}

void sx1236_configure_rx(SPIDriver * spip, struct CONFIG_SX1236_RX * c)
{
	sx1236_write_reg(spip,          regaddrs.RegOpMode, c->RegOpMode);
	sx1236_write_carrier_freq(spip, c->carrier_freq,    c->Fstep);
	sx1236_set_freq_deviation(spip, c->freq_dev_hz,     c->Fstep );
	sx1236_set_bitrate(spip,        c->Fxosc,           c->bitrate);

	sx1236_write_reg(spip, regaddrs.RegPacketConfig1,   c->RegPacketConfig1);
	sx1236_write_reg(spip, regaddrs.RegPacketConfig2,   c->RegPacketConfig2);
	sx1236_write_reg(spip, regaddrs.RegPaRamp,          c->RegPaRamp);
	sx1236_write_reg(spip, regaddrs.RegPll,             c->RegPllLf);
	sx1236_write_reg(spip, regaddrs.RegRssiThresh,      c->RegRssiThresh);
	sx1236_write_reg(spip, regaddrs.RegSyncConfig,      c->RegSyncConfig);

	sx1236_write_reg(spip, regaddrs.RegSyncValue1,      c->RegSyncValue1);
	sx1236_write_reg(spip, regaddrs.RegSyncValue2,      c->RegSyncValue2);
	sx1236_write_reg(spip, regaddrs.RegSyncValue3,      c->RegSyncValue3);
	sx1236_write_reg(spip, regaddrs.RegSyncValue4,      c->RegSyncValue4);
	sx1236_write_reg(spip, regaddrs.RegSyncValue5,      c->RegSyncValue5);
	sx1236_write_reg(spip, regaddrs.RegSyncValue6,      c->RegSyncValue6);
	sx1236_write_reg(spip, regaddrs.RegSyncValue7,      c->RegSyncValue7);
	sx1236_write_reg(spip, regaddrs.RegSyncValue8,      c->RegSyncValue8);

	sx1236_write_reg(spip, regaddrs.RegPayloadLength,   c->RegPayloadLength);
	sx1236_write_reg(spip, regaddrs.RegFifoThresh,      c->RegFifoThresh);
	sx1236_write_reg(spip, regaddrs.RegRxConfig,        c->RegRxConfig);
	sx1236_write_reg(spip, regaddrs.RegAfcFei,          c->RegAfcFei);


	// Sequencer states 4.1.8.2?

	// Development only: Check register writes
	sx1236_check_reg(spip, regaddrs.RegOpMode,          c->RegOpMode);
	sx1236_check_reg(spip, regaddrs.RegPacketConfig1,   c->RegPacketConfig1);
	sx1236_check_reg(spip, regaddrs.RegPacketConfig2,   c->RegPacketConfig2);
	sx1236_check_reg(spip, regaddrs.RegPaRamp,          c->RegPaRamp);
	sx1236_check_reg(spip, regaddrs.RegPll,             c->RegPllLf);
	sx1236_check_reg(spip, regaddrs.RegRssiThresh,      c->RegRssiThresh);
	sx1236_check_reg(spip, regaddrs.RegSyncConfig,      c->RegSyncConfig);
	sx1236_check_reg(spip, regaddrs.RegSyncValue1,      c->RegSyncValue1);
	sx1236_check_reg(spip, regaddrs.RegSyncValue2,      c->RegSyncValue2);
	sx1236_check_reg(spip, regaddrs.RegSyncValue3,      c->RegSyncValue3);
	sx1236_check_reg(spip, regaddrs.RegSyncValue4,      c->RegSyncValue4);
	sx1236_check_reg(spip, regaddrs.RegSyncValue5,      c->RegSyncValue5);
	sx1236_check_reg(spip, regaddrs.RegSyncValue6,      c->RegSyncValue6);
	sx1236_check_reg(spip, regaddrs.RegSyncValue7,      c->RegSyncValue7);
	sx1236_check_reg(spip, regaddrs.RegSyncValue8,      c->RegSyncValue8);

	sx1236_check_reg(spip, regaddrs.RegPayloadLength,   c->RegPayloadLength);
	sx1236_check_reg(spip, regaddrs.RegFifoThresh,      c->RegFifoThresh);
	sx1236_check_reg(spip, regaddrs.RegRxConfig,        c->RegRxConfig);
	sx1236_check_reg(spip, regaddrs.RegAfcFei,          c->RegAfcFei);

}





// ! @}

