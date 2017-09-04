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


// void config_sx1236(SPIDriver * spip, uint8_t OpModeCfg, uint8_t RegPAOutputCfg)
// {
// }

// void trans_write_register(uint8_t address, uint8_t * buffer, uint8_t length){
// [>
// This function initiates a SPI transaction for writing to
// the transceiver. It is assumed that the SPI0 module is in
// 8-bit mode.
// */

// [> Array for combining address and data <]
// uint8_t addr_buf[length + 1];

// [> Mask address byte with leading 1 for write <]
// addr_buf[0] = address | 0x80;

// [> copy data to remaining elements in array <]
// memcpy(addr_buf + 1, buffer, length);

// [> dummy array for receive part of transaction <]
// uint8_t recv[length + 1];

// [> conduct SPI transaction <]
// spi_transaction_8(&SPI0, (length + 1), addr_buf, recv);
// }

// #define LnaZin50 0x08
// #define RcCalStart (1 << 7)
// #define RcCalDone (1 << 6)

// #define DataModul_FSK 0x0
// #define PLLBandwidth_75kHz 0x0

// // RegOpMode
// #define SequencerOff (1 << 7)
// #define ListenAbort (1 << 5)
// #define ModeListen (1 << 6)
// #define ModeSleep (0 << 2)
// #define ModeStdby (1 << 2)
// #define ModeFS    (2 << 2)
// #define ModeTX    (3 << 2)
// #define ModeRX    (4 << 2)
// // RegDataModul
// #define Packet (0 << 5)
// #define Continuous (2 << 5)
// #define ContinuousNoSync (3 << 5)
// #define FSK (0 << 3)
// #define OOK (1 << 3)
// #define NoShaping (0 << 0)
// // RegAutoModes
// #define EnterNone         (0b000 << 5)
// #define EnterFifoNotEmpty (0b001 << 5)
// #define EnterFifoLevel    (0b010 << 5)
// #define EnterCrcOk        (0b011 << 5)
// #define EnterPayloadReady (0b100 << 5)
// #define EnterSyncAddress  (0b101 << 5)
// #define EnterPacketSent   (0b110 << 5)
// #define EnterFifoEmpty    (0b111 << 5)
// #define ExitNone         (0b000 << 2)
// #define ExitFifoEmpty    (0b001 << 2)
// #define ExitFifoLevel    (0b010 << 2)
// #define ExitCrcOk        (0b011 << 2)
// #define ExitPayloadReady (0b100 << 2)
// #define ExitSyncAddress  (0b101 << 2)
// #define ExitPacketSent   (0b110 << 2)
// #define ExitTimeout      (0b111 << 2)
// #define InterSleep (0b00 << 0)
// #define InterStdby (0b01 << 0)
// #define InterRX    (0b10 << 0)
// #define InterTX    (0b11 << 0)
// // RegSyncConfig
// #define SyncOn (1 << 7)
// #define FifoFillSyncAddress (0 << 6)
// #define FifoFillCondition (1 << 6)
// #define SyncSize(bytes) ((((bytes) - 1) & 0x7) << 3)
// #define SyncTol(errors) ((errors) & 0x7)
// // RegAfcFei
// #define AfcAutoOn (1 << 2)
// #define AfcAutoclearOn (1 << 3)



// #define FXOSC (32000000) //32MHz
// #define Fstep (FXOSC/(1<<19))
// #define FstepMul ((uint64_t)(1 << 8))
// #define FstepDiv ((uint64_t)(15625))

// static void setCarrierFrequency(uint32_t carrierHz) {
// uint64_t frf = ((uint64_t)carrierHz * FstepMul) / FstepDiv;
// uint8_t RegFrf[3] = {(frf >> 16) & 0xff, (frf >> 8) & 0xff, frf & 0xff};
// trans_write_register(transceiver.RegFrfMsb, RegFrf, 3);
// }

// static void setFrequencyDeviation(uint32_t deviationHz) {
// uint64_t fdev = ((uint64_t)deviationHz * FstepMul) / FstepDiv;
// uint8_t RegFdev[2] = {(fdev >> 8) & 0x3F, fdev & 0xFF};
// trans_write_register(transceiver.RegFdevMsb, RegFdev, 2);
// }

// static void setBitrate(uint32_t bitrateHz) {
// uint16_t rate = FXOSC/bitrateHz;
// uint8_t RegBitrate[2] = {(rate >> 8) & 0xff, rate & 0xff};
// trans_write_register(transceiver.RegBitrateMsb, RegBitrate, 2);
// }

// void configure_transceiver(uint8_t OpModeCfg, uint8_t RegPAOutputCfg){
// /* Change to frequency synthesizer mode */
// trans_write_register(transceiver.RegOpMode, (uint8_t[]){ModeFS}, 1);
// /* turn modulation to frequency shift keying */
// setCarrierFrequency(436500000);
// setFrequencyDeviation(2500);
// setBitrate(2400);

// trans_write_register(transceiver.RegDataModul, (uint8_t[]){Packet | FSK | NoShaping}, 1);
// /* adjust PLL bandwidth to 75kHz */
// trans_write_register(transceiver.RegTestPLL, (uint8_t[]){PLLBandwidth_75kHz}, 1);
// /* set LNA's input impedance to 50 ohm */
// trans_write_register(transceiver.RegLna, (uint8_t[]){LnaZin50}, 1);

// /* configure PA output power */
// trans_write_register(transceiver.RegPaLevel, (uint8_t[]){RegPAOutputCfg}, 1);

// uint8_t autoModes = 0;
// if(OpModeCfg == Mode_TX)
// autoModes = EnterFifoNotEmpty | InterTX | ExitPacketSent;
// else
// autoModes = EnterNone | InterSleep | ExitNone;
// //autoModes = EnterSyncAddress | InterRX | ExitFifoNotEmpty;
// //autoModes = EnterPayloadReady | InterSleep | ExitFifoEmpty;
// trans_write_register(transceiver.RegAutoModes, &autoModes, 1);

// trans_write_register(transceiver.RegRxBw, (uint8_t[]){0x55}, 1);
// trans_write_register(transceiver.RegRssiThresh, (uint8_t[]){0x70}, 1);

// uint8_t SyncConfig = SyncOn | FifoFillSyncAddress | SyncSize(1) | SyncTol(0);
// trans_write_register(transceiver.RegSyncConfig, &SyncConfig, 1);

// /*Sync word setup*/
// trans_write_register(transceiver.RegSyncValue1, (uint8_t[]){0xE7, 0xE7, 0xE7, 0xE7}, 4);

// /*Setup the packet config: no encoding no crc*/
// trans_write_register(transceiver.RegPacketConfig1, (uint8_t[]){0x08}, 1);

// //Sets preamble size
// trans_write_register(transceiver.RegPreambleMsb, (uint8_t[]){0x00, 0x10}, 2);

// //Sets the payload length
// /*
// The payload length needs to be equal to the buffer of data to be sent
// when the tx ready signal is produces on fifo_not_empty. If the tx
// ready signal is received from a fifo threshold reached condition
// then the payload length needs to be the same as the fifo threshold and
// the buffer needs to be one larger than the payload size.

// When using auto modes be sure to set the transceiver into standby mode
// it will wake and do its thing automagically.

//
// * trans_write_register(transceiver.RegPayloadLength, (uint8_t[]){5}, 1);
// *
// * [> To trigger on a fifo threshhold set RegFifoThresh to PACKET_LENGTH<]
// * [> Trigger on fifo not empty <]
// * trans_write_register(transceiver.RegFifoThresh, (uint8_t[]){0x04}, 1);
// *
// * trans_write_register(transceiver.RegAfcFei, (uint8_t[]){AfcAutoOn | AfcAutoclearOn}, 1);
// *
// * [> Set transceiver mode <]
// * trans_write_register(transceiver.RegOpMode, (uint8_t[]){OpModeCfg}, 1);
// * }
// *
// *
// * ! @}
// *
// *

