/*
	Header file for transceiver driver.
	Contains a struct that holds the addresses for registers
	in the transceiver block. Also contains basic functions
	for utilizing the transceiver.

	Programmed by William Harrington and Michael Mathis for 
	the low-gain-radio capstone 2016.

	Modified by Evan Yand for C3 board 2017. 
	Note: Further edits may be needed.
*/
#include <stdbool.h>
#include <stdint.h>


#ifndef _TRANSCEIVER_H_
#define _TRANSCEIVER_H_

#define PACKET_LENGTH 5

/* struct to hold transceiver register addresses*/
struct SX1236 {
  	uint8_t RegFifo; 				/* FIFO read/write access */
  	uint8_t RegOpMode; 				/* Operating modes of the transceiver */
  	uint8_t RegBitrateMsb; 			/* bit rate setting, most significant bits */
  	uint8_t RegBitrateLsb; 			/* bit rate setting, least significant bits */
  	uint8_t RegFdevMsb; 			/* frequency deviation setting, most significant bits */
  	uint8_t RegFdevLsb; 			/* frequency deviation setting, least significant bits */
  	uint8_t RegFrfMsb; 				/* RF carrier frequency, most significant bits */
	uint8_t RegFrfMid; 				/* RF carrier frequency, Intermediate Bits */
	uint8_t RegFrfLsb; 				/* RF carrier frequency, least significant bits */
	uint8_t RegPaConfig;     		/* PA selection and output power control */
	uint8_t RegPaRamp; 				/* Control of PA ramp*/
	uint8_t RegOcp; 				/* Over Current Protection control */
	uint8_t RegLna; 				/* LNA settings */
	uint8_t RegRxConfig;			/* AFC, AGC, ctrl */
	uint8_t RegRssiConfig; 			/* RSSI-related settings */
	uint8_t RegRssiCollision;		/* RSSI collision detector */
	uint8_t RegRssiThresh; 			/* RSSI Threshold control */
	uint8_t RegRssiValue; 			/* RSSI value in dBm */	
	uint8_t RegRxBw; 				/* Channel Filter BW Control */
	uint8_t RegAfcBw; 				/* Channel Filter BW control during the AFC routine */
	uint8_t RegOokPeak; 			/* OOK demodulator selection and control in peak mode */
	uint8_t RegOokFix; 				/* Fixed threshold control of the OOK demodulator */
	uint8_t RegOokAvg; 				/* Average threshold control of the OOK demodulator */
	uint8_t RegAfcFei; 				/* AFC and FEI control and status */
	uint8_t RegAfcMsb; 				/* MSB of the frequency correction of the AFC */
	uint8_t RegAfcLsb; 				/* LSB of the frequency correction of the AFC */
	uint8_t RegFeiMsb; 				/* MSB of the calculated frequency error */
	uint8_t RegFeiLsb; 				/* LSB of the calculated frequency error */
	uint8_t RegPreambleDetect;		/* Settings of preamble detector */
	uint8_t RegRxTimeout1; 			/* Timeout duration between RX request and RSSI detection */
	uint8_t RegRxTimeout2; 			/* Timeout duration between RSSI detection and PayloadReady */
	uint8_t RegRxTimeout3; 			/* Timeout duration between RX request and RSSI detection */
	uint8_t RegRxDelay;				/* Delay between Rx cycles */
	uint8_t RegOsc;					/* RC oscillator settings, CLKOUT frequency */
	uint8_t RegPreambleMsb;			/* Preamble length, MSB */
	uint8_t RegPreambleLsb;			/* Preamble length, LSB */
	uint8_t RegSyncConfig; 			/* Sync Word Recognition control */
	uint8_t RegSyncValue1;			/* Sync Word byte 1 */
	uint8_t RegSyncValue2; 			/* Sync Word byte 2 */
	uint8_t RegSyncValue3; 			/* Sync Word byte 3 */
	uint8_t RegSyncValue4; 			/* Sync Word byte 4 */
	uint8_t RegSyncValue5; 			/* Sync Word byte 5 */
	uint8_t RegSyncValue6; 			/* Sync Word byte 6 */
	uint8_t RegSyncValue7; 			/* Sync Word byte 7 */
	uint8_t RegSyncValue8; 			/* Sync Word byte 8 */
	uint8_t RegPacketConfig1; 		/* Packet mode settings */
	uint8_t RegPacketConfig2; 		/* Packet mode settings */
	uint8_t RegPayloadLength; 		/* Payload length setting */
	uint8_t RegNodeAdrs; 			/* Node address */
	uint8_t RegBroadcastAdrs; 		/* Broadcast address */
	uint8_t RegFifoThresh; 			/* Fifo threshold, TX start condition */
	uint8_t RegSeqConfig1;
	uint8_t RegSeqConfig2;
	uint8_t RegTimerResol;
	uint8_t RegTimer1Coef;
	uint8_t RegTimer2Coef;
	uint8_t RegImageCal;
	uint8_t RegTemp;
	uint8_t RegLowBat;	
	uint8_t RegIrqFlags1; 			/* Status register: PLL Lock state, Timeout, RSSI > Threshold... */
	uint8_t RegIrqFlags2; 			/* Status register: FIFO handling flags, Low battery detection... */
	uint8_t RegDioMapping1; 		/* Mapping of pins DIO0 to DIO3 */
	uint8_t RegDioMapping2; 		/* Mapping of pins DIO4 and DIO5/Clkout frequency */
	uint8_t RegVersion;				/* RF ID relating the silicon revision */
	uint8_t RegPllHop;				/* Control the fast frequency hopping module */
	uint8_t RegTcxo;				/* TCXO or XTAL input setting */
	uint8_t RegPaDac;				/* Higher power settings of the PA */
	uint8_t RegFormerTemp;			/* Stored temp during the former IQ Calibration */
	uint8_t RegBitRateFrac;			/* Fractional part in the bitrate division ratio */
	uint8_t RegAgcRef;				/* Adjustment of the AGC thresholds */
	uint8_t RegAgcThresh1;			/************************************/
	uint8_t RegAgcThresh2;			/************************************/
	uint8_t RegAgcThresh3;			/************************************/
	uint8_t RegPll;					/* Control of the PLL bandwidth */
	
};

/* allow for external access to struct */
extern struct SX1236 transceiver;

#define Mode_RX (1 << 4)
#define Mode_TX (3 << 2)

#define PA0 (1 << 7)
#define PA1 (1 << 6)
#define PA2 (1 << 5)

#define PAOutputCfg(pa, power) (((pa) & (PA0 | PA1 | PA2)) | ((power) & 0x1F))

// RegIrqFlags1
#define ModeReady (1 << 7)
#define RxReady (1 << 6)
#define TxReady (1 << 5)
#define PllLock (1 << 4)
#define Rssi (1 << 3)
#define Timeout (1 << 2)
#define AutoMode (1 << 1)
#define SyncAddressMatch (1 << 0)
// RegIrqFlags2
#define FifoFull (1 << 7)
#define FifoNotEmpty (1 << 6)
#define FifoLevel (1 << 5)
#define FifoOverrun (1 << 4)
#define PacketSent (1 << 3)
#define PayloadReady (1 << 2)
#define CrcOk (1 << 1)
#define LowBat (1 << 0)

void initialize_trans_spi(volatile struct spi * SPI);
void trans_read_register(uint8_t address, uint8_t * buffer, uint8_t length);
void trans_write_register(uint8_t address, uint8_t * buffer, uint8_t length);
void configure_transceiver(uint8_t OpModeCfg, uint8_t RegPAOutputCfg);

#endif
