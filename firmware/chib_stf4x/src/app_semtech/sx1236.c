/*
	Implementation file for transceiver driver.
	Contains a struct that holds the addresses for registers
	in the transceiver block. Also contains basic functions
	for utilizing the transceiver.

	Programmed by William Harrington and Michael Mathis for 
	the low-gain-radio capstone 2016.

	Modified by Evan Yand for C3 board 2017. 
	Note: Further edits may be needed.
 */
//#include "transceiver.h"
#include "spi.h"
#include <string.h>
//#include "sx1236.h"
#include "semtech-dev-board-registers.h"


void trans_read_register(uint8_t address, uint8_t * buffer, uint8_t length){
	/*
	   This function initiates a SPI transaction for reading
	   a register(s) in the transceiver block. It is assumed that
	   the SPI0 module is in 8-bit mode.
	*/

	/* We need to send arbitrary values after the address byte 
		because the transceiver autoincrements the address */
	uint8_t write_data[length + 1];
	uint8_t recv[length + 1];

	write_data[0] = address;
	memset(write_data + 1, 0, length);

	spi_transaction(&SPI0, length + 1, write_data, recv);

	/* Copy the received data back into the user's buffer */
	memcpy(buffer, recv + 1, length);

}

void trans_write_register(uint8_t address, uint8_t * buffer, uint8_t length){
	/*
	   This function initiates a SPI transaction for writing to
	   the transceiver. It is assumed that the SPI0 module is in
	   8-bit mode.
	*/

	/* Array for combining address and data */
	uint8_t addr_buf[length + 1];

	/* Mask address byte with leading 1 for write */
	addr_buf[0] = address | 0x80;

	/* copy data to remaining elements in array */
	memcpy(addr_buf + 1, buffer, length);

	/* dummy array for receive part of transaction */ 
	uint8_t recv[length + 1];

	/* conduct SPI transaction */
	spi_transaction_8(&SPI0, (length + 1), addr_buf, recv);
}

#define LnaZin50 0x08
#define RcCalStart (1 << 7)
#define RcCalDone (1 << 6)

#define DataModul_FSK 0x0
#define PLLBandwidth_75kHz 0x0

// RegOpMode
#define SequencerOff (1 << 7)
#define ListenAbort (1 << 5)
#define ModeListen (1 << 6)
#define ModeSleep (0 << 2)
#define ModeStdby (1 << 2)
#define ModeFS    (2 << 2)
#define ModeTX    (3 << 2)
#define ModeRX    (4 << 2)
// RegDataModul
#define Packet (0 << 5)
#define Continuous (2 << 5)
#define ContinuousNoSync (3 << 5)
#define FSK (0 << 3)
#define OOK (1 << 3)
#define NoShaping (0 << 0)
// RegAutoModes
#define EnterNone         (0b000 << 5)
#define EnterFifoNotEmpty (0b001 << 5)
#define EnterFifoLevel    (0b010 << 5)
#define EnterCrcOk        (0b011 << 5)
#define EnterPayloadReady (0b100 << 5)
#define EnterSyncAddress  (0b101 << 5)
#define EnterPacketSent   (0b110 << 5)
#define EnterFifoEmpty    (0b111 << 5)
#define ExitNone         (0b000 << 2)
#define ExitFifoEmpty    (0b001 << 2)
#define ExitFifoLevel    (0b010 << 2)
#define ExitCrcOk        (0b011 << 2)
#define ExitPayloadReady (0b100 << 2)
#define ExitSyncAddress  (0b101 << 2)
#define ExitPacketSent   (0b110 << 2)
#define ExitTimeout      (0b111 << 2)
#define InterSleep (0b00 << 0)
#define InterStdby (0b01 << 0)
#define InterRX    (0b10 << 0)
#define InterTX    (0b11 << 0)
// RegSyncConfig
#define SyncOn (1 << 7)
#define FifoFillSyncAddress (0 << 6)
#define FifoFillCondition (1 << 6)
#define SyncSize(bytes) ((((bytes) - 1) & 0x7) << 3)
#define SyncTol(errors) ((errors) & 0x7)
// RegAfcFei
#define AfcAutoOn (1 << 2)
#define AfcAutoclearOn (1 << 3)



#define FXOSC (32000000) //32MHz
#define Fstep (FXOSC/(1<<19))
#define FstepMul ((uint64_t)(1 << 8))
#define FstepDiv ((uint64_t)(15625))

static void setCarrierFrequency(uint32_t carrierHz) {
	uint64_t frf = ((uint64_t)carrierHz * FstepMul) / FstepDiv;
	uint8_t RegFrf[3] = {(frf >> 16) & 0xff, (frf >> 8) & 0xff, frf & 0xff};
	trans_write_register(transceiver.RegFrfMsb, RegFrf, 3);
}

static void setFrequencyDeviation(uint32_t deviationHz) {
	uint64_t fdev = ((uint64_t)deviationHz * FstepMul) / FstepDiv;
	uint8_t RegFdev[2] = {(fdev >> 8) & 0x3F, fdev & 0xFF};
	trans_write_register(transceiver.RegFdevMsb, RegFdev, 2);
}

static void setBitrate(uint32_t bitrateHz) {
	uint16_t rate = FXOSC/bitrateHz;
	uint8_t RegBitrate[2] = {(rate >> 8) & 0xff, rate & 0xff};
	trans_write_register(transceiver.RegBitrateMsb, RegBitrate, 2);
}

void configure_transceiver(uint8_t OpModeCfg, uint8_t RegPAOutputCfg){
	/* Change to frequency synthesizer mode */
	trans_write_register(transceiver.RegOpMode, (uint8_t[]){ModeFS}, 1);
	/* turn modulation to frequency shift keying */
	setCarrierFrequency(436500000);
	setFrequencyDeviation(2500);
	setBitrate(2400);

	trans_write_register(transceiver.RegDataModul, (uint8_t[]){Packet | FSK | NoShaping}, 1);
	/* adjust PLL bandwidth to 75kHz */
	trans_write_register(transceiver.RegTestPLL, (uint8_t[]){PLLBandwidth_75kHz}, 1);
	/* set LNA's input impedance to 50 ohm */
	trans_write_register(transceiver.RegLna, (uint8_t[]){LnaZin50}, 1);
	
	/* configure PA output power */
	trans_write_register(transceiver.RegPaLevel, (uint8_t[]){RegPAOutputCfg}, 1);

	uint8_t autoModes = 0;
	if(OpModeCfg == Mode_TX)
		autoModes = EnterFifoNotEmpty | InterTX | ExitPacketSent;
	else
		autoModes = EnterNone | InterSleep | ExitNone;
		//autoModes = EnterSyncAddress | InterRX | ExitFifoNotEmpty;
		//autoModes = EnterPayloadReady | InterSleep | ExitFifoEmpty;
	trans_write_register(transceiver.RegAutoModes, &autoModes, 1);

	trans_write_register(transceiver.RegRxBw, (uint8_t[]){0x55}, 1);
	trans_write_register(transceiver.RegRssiThresh, (uint8_t[]){0x70}, 1);

	uint8_t SyncConfig = SyncOn | FifoFillSyncAddress | SyncSize(1) | SyncTol(0);
	trans_write_register(transceiver.RegSyncConfig, &SyncConfig, 1);

	/*Sync word setup*/
	trans_write_register(transceiver.RegSyncValue1, (uint8_t[]){0xE7, 0xE7, 0xE7, 0xE7}, 4);

	/*Setup the packet config: no encoding no crc*/
	trans_write_register(transceiver.RegPacketConfig1, (uint8_t[]){0x08}, 1);

	//Sets preamble size
	trans_write_register(transceiver.RegPreambleMsb, (uint8_t[]){0x00, 0x10}, 2);

	//Sets the payload length
	/*
		The payload length needs to be equal to the buffer of data to be sent 
		when the tx ready signal is produces on fifo_not_empty. If the tx 
		ready signal is received from a fifo threshold reached condition
		then the payload length needs to be the same as the fifo threshold and 
		the buffer needs to be one larger than the payload size.

		When using auto modes be sure to set the transceiver into standby mode
		it will wake and do its thing automagically.

	*/
	trans_write_register(transceiver.RegPayloadLength, (uint8_t[]){5}, 1);

	/* To trigger on a fifo threshhold set RegFifoThresh to PACKET_LENGTH*/
	/* Trigger on fifo not empty */
	trans_write_register(transceiver.RegFifoThresh, (uint8_t[]){0x04}, 1);

	trans_write_register(transceiver.RegAfcFei, (uint8_t[]){AfcAutoOn | AfcAutoclearOn}, 1);

	/* Set transceiver mode */
	trans_write_register(transceiver.RegOpMode, (uint8_t[]){OpModeCfg}, 1);
}
