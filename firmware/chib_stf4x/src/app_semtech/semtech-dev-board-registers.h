//
//  Register values needed for initial testing.
//  G.N. LeBrasseur  1-Sep-2017
//
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


#define SEMTECH_REGISTERS\
	X(RegFifo)\
	X(RegOpMode)\
	X(RegBitrateMsb)\
	X(RegBitrateLsb)\
	X(RegFdevMsb)\
	X(RegFdevLsb)\
	X(RegFrfMsb)\
	X(RegFrfMid)\
	X(RegFrfLsb)\
	X(RegPaConfig)\
	X(RegPaRamp)\
	X(RegOcp)\
	X(RegLna)\
	X(RegRxConfig)\
	X(RegRssiConfig)\
	X(RegRssiCollision)\
	X(RegRssiThresh)\
	X(RegRssiValue)\
	X(RegRxBw)\
	X(RegAfcBw)\
	X(RegOokPeak)\
	X(RegOokFix)\
	X(RegOokAvg)\
	X(RegAfcFei)\
	X(RegAfcMsb)\
	X(RegAfcLsb)\
	X(RegPreambleDetect)\
	X(RegRxTimeout1)\
	X(RegRxTimeout2)\
	X(RegRxTimeout3)\
	X(RegRxDelay)\
	X(RegOsc)\
	X(RegPreambleMsb)\
	X(RegPreambleLsb)\
	X(RegSyncConfig)\
	X(RegSyncValue1)\
	X(RegSyncValue2)\
	X(RegSyncValue3)\
	X(RegSyncValue4)\
	X(RegSyncValue5)\
	X(RegSyncValue6)\
	X(RegSyncValue7)\
	X(RegSyncValue8)\
	X(RegPacketConfig1)\
	X(RegPacketConfig2)\
	X(RegPayloadLength)\
	X(RegNodeAdrs)\
	X(RegBroadcastAdrs)\
	X(RegFifoThresh)\
	X(RegSeqConfig1)\
	X(RegSeqConfig2)\
	X(RegTimerResol)\
	X(RegTimer1Coef)\
	X(RegTimer2Coef)\
	X(RegImageCal)\
	X(RegTemp)\
	X(RegLowBat)\
	X(RegIrqFlags1)\
	X(RegIrqFlags2)\
	X(RegDioMapping1)\
	X(RegDioMapping2)\
	X(RegVersion)\
	X(RegPllHop)\
	X(RegTcxo)\
	X(RegPaDac)\
	X(RegFormerTemp)\
	X(RegBitRateFrac)\
	X(RegAgcRef)\
	X(RegAgcThresh1)\
	X(RegAgcThresh2)\
	X(RegAgcThresh3)\
	X(RegPll)

/* struct to hold transceiver register addresses*/


// Register addresses

struct SX1236 transceiver = {
	.RegFifo 		= 0x00,
	.RegOpMode 		= 0x01,
	.RegBitrateMsb 		= 0x02,
	.RegBitrateLsb 		= 0x03,
	.RegFdevMsb 		= 0x04,
	.RegFdevLsb 		= 0x05,
	.RegFrfMsb 		= 0x06,
	.RegFrfMid 		= 0x07,
	.RegFrfLsb 		= 0x08,
	.RegPaConfig 		= 0x09,
	.RegPaRamp 		= 0x0A,
	.RegOcp 		= 0x0B,
	.RegLna 		= 0x0C,
	.RegRxConfig 		= 0x0D,
	.RegRssiConfig 		= 0x0E,
	.RegRssiCollision 	= 0x0F,
	.RegRssiThresh 		= 0x10,
	.RegRssiValue 		= 0x11,
	.RegRxBw 		= 0x12,
	.RegAfcBw 		= 0x13,
	.RegOokPeak 		= 0x14,
	.RegOokFix 		= 0x15,
	.RegOokAvg 		= 0x16,
// Reserved 17, 18, 19
	.RegAfcFei 		= 0x1A,
	.RegAfcMsb 		= 0x1B,
	.RegAfcLsb 		= 0x1C,
	.RegFeiMsb 		= 0x1D,
	.RegFeiLsb 		= 0x1E,
	.RegPreambleDetect 	= 0x1F,
	.RegRxTimeout1 		= 0x20,
	.RegRxTimeout2 		= 0x21,
	.RegRxTimeout3 		= 0x22,
	.RegRxDelay 		= 0x23,
	.RegOsc 		= 0x24,
	.RegPreambleMsb 	= 0x25,
	.RegPreambleLsb 	= 0x26,
	.RegSyncConfig 		= 0x27,
	.RegSyncValue1 		= 0x28,
	.RegSyncValue2 		= 0x29,
	.RegSyncValue3 		= 0x2A,
	.RegSyncValue4 		= 0x2B,
	.RegSyncValue5 		= 0x2C,
	.RegSyncValue6 		= 0x2D,
	.RegSyncValue7 		= 0x2E,
	.RegSyncValue8 		= 0x2F,
	.RegPacketConfig1 	= 0x30,
	.RegPacketConfig2 	= 0x31,
	.RegPayloadLength 	= 0x32,
	.RegNodeAdrs 		= 0x33,
	.RegBroadcastAdrs 	= 0x34,
	.RegFifoThresh 		= 0x35,
	.RegSeqConfig1 		= 0x36,
	.RegSeqConfig2 		= 0x37,
	.RegTimerResol 		= 0x38,
	.RegTimer1Coef		= 0x39,
	.RegTimer2Coef		= 0x3A,
	.RegImageCal 		= 0x3B,
	.RegTemp 		= 0x3C,
	.RegLowBat 		= 0x3D,
	.RegIrqFlags1 		= 0x3E,
	.RegIrqFlags2 		= 0x3F,
	.RegDioMapping1 	= 0x40,
	.RegDioMapping2		= 0x41,
	.RegVersion 		= 0x42,
	.RegPllHop 		= 0x44,
// Skip 45, 46, 47, 48, 49, 4A
	.RegTcxo 		= 0x4B,
// Skip 4C
	.RegPaDac 		= 0x4D,
// Skip 4E through 5A
	.RegFormerTemp 		= 0x5B,
// Skip 5C
	.RegBitRateFrac 	= 0x5D,
// Skip 5E through 60
	.RegAgcRef 		= 0x61,
	.RegAgcThresh1 		= 0x62,
	.RegAgcThresh2 		= 0x63,
	.RegAgcThresh3 		= 0x64,
// Skip 65 through 6F
	.RegPll 		= 0x70
};


//  Default values set upon POR
//  These are forced by Semtech
//
//  Corrected some of the values to reflect POR values.
//  G.N. LeBrasseur  1-Sep-2017
//

struct SX1236 defaults = {
	.RegFifo 		= 0x00,
	.RegOpMode 		= 0x09,		// Was incorrect with 0x01
	.RegBitrateMsb 		= 0x1A,
	.RegBitrateLsb 		= 0x0B,
	.RegFdevMsb 		= 0x00,
	.RegFdevLsb 		= 0x52,
	.RegFrfMsb 		= 0x6C,
	.RegFrfMid 		= 0x80,
	.RegFrfLsb 		= 0x00,
	.RegPaConfig 		= 0x4F,
	.RegPaRamp 		= 0x09,
	.RegOcp 		= 0x2B,
	.RegLna 		= 0x20,
	.RegRxConfig 		= 0x0E,		// Was incorrect with 0x08
	.RegRssiConfig 		= 0x02,
	.RegRssiCollision 	= 0x0A,
	.RegRssiThresh 		= 0xFF,
	.RegRssiValue 		= 0x00,
	.RegRxBw 		= 0x15,
	.RegAfcBw 		= 0x0B,
	.RegOokPeak 		= 0x28,
	.RegOokFix 		= 0x0C,
	.RegOokAvg 		= 0x12,
// Reserved registers 17, 18, 19
	.RegAfcFei 		= 0x00,
	.RegAfcMsb 		= 0x00,
	.RegAfcLsb 		= 0x00,
	.RegFeiMsb 		= 0x00,
	.RegFeiLsb 		= 0x00,
	.RegPreambleDetect 	= 0xAA,		// Was incorrect with 0x40
	.RegRxTimeout1 		= 0x00,
	.RegRxTimeout2 		= 0x00,
	.RegRxTimeout3 		= 0x00,
	.RegRxDelay 		= 0x00,
	.RegOsc 		= 0x07,		// Was incorrct with 0x05
	.RegPreambleMsb 	= 0x00,
	.RegPreambleLsb 	= 0x03,
	.RegSyncConfig 		= 0x93,
	.RegSyncValue1 		= 0x01,		// Was incorrect with 0x55
	.RegSyncValue2 		= 0x01,		// Was incorrect with 0x55
	.RegSyncValue3 		= 0x01,		// Was incorrect with 0x55
	.RegSyncValue4 		= 0x01,		// Was incorrect with 0x55
	.RegSyncValue5 		= 0x01,		// Was incorrect with 0x55
	.RegSyncValue6 		= 0x01,		// Was incorrect with 0x55
	.RegSyncValue7 		= 0x01,		// Was incorrect with 0x55
	.RegSyncValue8 		= 0x01,		// Was incorrect with 0x55
	.RegPacketConfig1 	= 0x90,
	.RegPacketConfig2 	= 0x40,
	.RegPayloadLength 	= 0x40,
	.RegNodeAdrs 		= 0x00,
	.RegBroadcastAdrs 	= 0x00,
	.RegFifoThresh 		= 0x8F,		// Was incorrect with 0x0F
	.RegSeqConfig1 		= 0x00,
	.RegSeqConfig2 		= 0x00,
	.RegTimerResol 		= 0x00,
	.RegTimer1Coef		= 0xF5,
	.RegTimer2Coef		= 0x20,
	.RegImageCal 		= 0x02,		// Was incorrect with 0x82
	.RegTemp 		= 0x00,
	.RegLowBat 		= 0x02,
	.RegIrqFlags1 		= 0x00,		// Was incorrect with 0x80
	.RegIrqFlags2 		= 0x00,		// Was incorrect with 0x40 (possiblly in error?)
	.RegDioMapping1 	= 0x00,
	.RegDioMapping2		= 0x00,
	.RegVersion 		= 0x00,		// Was incorrect with 0x12 (read only value should be 0x12)
	.RegPllHop 		= 0x2D,
	.RegTcxo 		= 0x09,
	.RegPaDac 		= 0x84,
	.RegFormerTemp 		= 0x00,
	.RegBitRateFrac 	= 0x00,
//  Band specific additional registers; Two copies depending on RegOpMode.LowFrequencyModeOn; POR default is LF (i.e. 1)
	.RegAgcRef 		= 0x19,		// Was incorrect with 0x13
	.RegAgcThresh1 		= 0x0C,		// Was incorrect with 0x0E
	.RegAgcThresh2 		= 0x4B,		// Was incorrect with 0x5B
	.RegAgcThresh3 		= 0xCC,		// Was incorrect with 0xDB
	.RegPll 		= 0xD0
};



// Intended setup values after POR and needing to be set for OreSat testing.
//
// NB:  Values marked with an "*" change from POR default
//

struct SX1236 intended_setup = {
	.RegFifo 		= 0x00,		//
	.RegOpMode 		= 0x09,		// *  Bit field portion called Mode (2-0) will change as transceiver is used
	.RegBitrateMsb 		= 0x06,		// *  Set BR to 19k2 bps
	.RegBitrateLsb 		= 0x83,		// *
	.RegFdevMsb 		= 0x01,		// *  Set FDEV t0 20 kHz, so Fdev(13,0) = 0x0148
	.RegFdevLsb 		= 0x48,		// *
	.RegFrfMsb 		= 0x6D,		// *  Set RF Frequency to 436.5 MHz
	.RegFrfMid 		= 0x20,		// *
	.RegFrfLsb 		= 0x00,		//
	.RegPaConfig 		= 0x00,		// *  Set transmitter power to -4.2 dBm
	.RegPaRamp 		= 0x09,		//
	.RegOcp 		= 0x2B,		//
	.RegLna 		= 0x20,		//
	.RegRxConfig 		= 0x08,		//
	.RegRssiConfig 		= 0x02,		//
	.RegRssiCollision 	= 0x0A,		//
	.RegRssiThresh 		= 0xFF,		//
	.RegRssiValue 		= 0x00,		//
	.RegRxBw 		= 0x15,		//
	.RegAfcBw 		= 0x0B,		//
	.RegOokPeak 		= 0x28,		//
	.RegOokFix 		= 0x0C,		//
	.RegOokAvg 		= 0x12,		//
	.RegAfcFei 		= 0x00,		//
	.RegAfcMsb 		= 0x00,		//
	.RegAfcLsb 		= 0x00,		//
	.RegFeiMsb 		= 0x00,		//
	.RegFeiLsb 		= 0x00,		//
	.RegPreambleDetect 	= 0x40,		//
	.RegRxTimeout1 		= 0x00,		//
	.RegRxTimeout2 		= 0x00,		//
	.RegRxTimeout3 		= 0x00,		//
	.RegRxDelay 		= 0x00,		//
	.RegOsc 		= 0x05,		//
	.RegPreambleMsb 	= 0x00,		//
	.RegPreambleLsb 	= 0x03,		//
	.RegSyncConfig 		= 0x93,		//
	.RegSyncValue1 		= 0x55,		//
	.RegSyncValue2 		= 0x55,		//
	.RegSyncValue3 		= 0x55,		//
	.RegSyncValue4 		= 0x55,		//
	.RegSyncValue5 		= 0x55,		//
	.RegSyncValue6 		= 0x55,		//
	.RegSyncValue7 		= 0x55,		//
	.RegSyncValue8 		= 0x55,		//
	.RegPacketConfig1 	= 0x90,		//
	.RegPacketConfig2 	= 0x40,		//
	.RegPayloadLength 	= 0x40,		//
	.RegNodeAdrs 		= 0x00,		//
	.RegBroadcastAdrs 	= 0x00,		//
	.RegFifoThresh 		= 0x0F,		//
	.RegSeqConfig1 		= 0x00,		//
	.RegSeqConfig2 		= 0x00,		//
	.RegTimerResol 		= 0x00,		//
	.RegTimer1Coef		= 0xF5,		//
	.RegTimer2Coef		= 0x20,		//
	.RegImageCal 		= 0x82,		//
	.RegTemp 		= 0x00,		//
	.RegLowBat 		= 0x02,		//
	.RegIrqFlags1 		= 0x80,		//
	.RegIrqFlags2 		= 0x40,		//
	.RegDioMapping1 	= 0x00,		//
	.RegDioMapping2		= 0x00,		//
	.RegVersion 		= 0x12,		//
	.RegPllHop 		= 0x2D,		//
	.RegTcxo 		= 0x09,		//
	.RegPaDac 		= 0x84,		//
	.RegFormerTemp 		= 0x00,		//
	.RegBitRateFrac 	= 0x00,		//
	.RegAgcRef 		= 0x13,		//
	.RegAgcThresh1 		= 0x0E,		//
	.RegAgcThresh2 		= 0x5B,		//
	.RegAgcThresh3 		= 0xDB,		//
	.RegPll 		= 0xD0		//
};

