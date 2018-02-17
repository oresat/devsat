/*! \file sx1236.h */

/*!
 * \addtogroup sx1236
 *  * @{
 *
 */

#ifndef _SX1236_H
#define _SX1236_H

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

#include <stdbool.h>
#include <stdint.h>

#include "util_numbers.h"

/*
 * sx1231 RegOpMode
 * sx1231 Datasheet p 65
 */
#define SX1236_MODE_MASK             ((uint8_t)(0xF8))
#define SX1236_FSK_MODE              ((uint8_t)(0b00<<5))
#define SX1236_OOK_MODE              ((uint8_t)(0b01<<5))
#define SX1236_LOW_FREQ_MODE         ((uint8_t)(0b01<<3))

#define SX1236_SLEEP_MODE            ((uint8_t)(0b000<<0))
#define SX1236_STANDBY_MODE          ((uint8_t)(0b001<<0))
#define SX1236_FS_MODE_TX            ((uint8_t)(0b010<<0))
#define SX1236_TRANSMITTER_MODE      ((uint8_t)(0b011<<0))
#define SX1236_FS_MODE_RX            ((uint8_t)(0b100<<0))
#define SX1236_RECEIVER_MODE         ((uint8_t)(0b101<<0))

// RegPaRamp
#define SX1236_NO_SHAPING            ((uint8_t)(0b00<<5))


// RegDioMapping1
#define SX1236_DIO0_PAYLOAD_RDY      ((uint8_t)(0b00<<6))
#define SX1236_DIO0_CRC_OK           ((uint8_t)(0b01<<6))

#define SX1236_DIO2_RX_RDY           ((uint8_t)(0b01<<2))

// RegDioMapping2
#define SX1236_DIO4_RX_TIMEOUT       ((uint8_t)(0b10<<6))
#define SX1236_DIO5_CLKOUT           ((uint8_t)(0b00<<4))

// Packet Config 1
#define SX1236_FIXED_PACKET          ((uint8_t)(0b0<<7))
#define SX1236_VARIABLE_PACKET       ((uint8_t)(0b1<<7))

#define SX1236_DCFREE_NONE       	 ((uint8_t)(0b00<<5))
#define SX1236_DCFREE_MANCHESTER   	 ((uint8_t)(0b00<<5))
#define SX1236_DCFREE_WHITENING      ((uint8_t)(0b00<<5))

#define SX1236_CRC_OFF		         ((uint8_t)(0b0<<4))
#define SX1236_CRC_ON      			 ((uint8_t)(0b1<<4))

// Packet Config 2
#define SX1236_CONTINUOUS_MODE       ((uint8_t)(0b0<<6))
#define SX1236_PACKET_MODE           ((uint8_t)(0b1<<6))


// PllLf
#define SX1236_PLLBW_75KHZ           ((uint8_t)(0b00<<6))
#define SX1236_PLLBW_150KHZ          ((uint8_t)(0b01<<6))
#define SX1236_PLLBW_225KHZ          ((uint8_t)(0b10<<6))
#define SX1236_PLLBW_300KHZ          ((uint8_t)(0b11<<6))

// SyncConfig
#define SX1236_SYNC_ON               ((uint8_t)(0b1<<4))
#define SX1236_SYNC_OFF              ((uint8_t)(0b0<<4))

// RxConfig
#define SX1236_RESTART_RX_WITH_PLL_LOCK  ((uint8_t)(0b1<<5))
#define SX1236_AFC_AUTO_ON           ((uint8_t)(0b1<<4))
#define SX1236_AGC_AUTO_ON           ((uint8_t)(0b1<<3))
#define SX1236_RXTRIGGER_ALL         ((uint8_t)(0b111<<0))
// AfcFei
#define SX1236_AFC_AUTO_CLEAR_ON     ((uint8_t)(0b1<<0))

// Osc
#define SX1236_OSC_DIV_1             ((uint8_t)(0b000<<0))
#define SX1236_OSC_DIV_8             ((uint8_t)(0b011<<0))
#define SX1236_OSC_OFF               ((uint8_t)(0b111<<0))

// OOk peak
#define SX1236_BIT_SYNC_ON           ((uint8_t)(0b1<<5))

/*
 * struct to hold transceiver register addresses
 */
struct SX1236
{
	uint8_t RegFifo;                /* FIFO read/write access */
	uint8_t RegOpMode;              /* Operating modes of the transceiver */
	uint8_t RegBitrateMsb;          /* bit rate setting, most significant bits */
	uint8_t RegBitrateLsb;          /* bit rate setting, least significant bits */
	uint8_t RegFdevMsb;             /* frequency deviation setting, most significant bits */
	uint8_t RegFdevLsb;             /* frequency deviation setting, least significant bits */
	uint8_t RegFrfMsb;              /* RF carrier frequency, most significant bits */
	uint8_t RegFrfMid;              /* RF carrier frequency, Intermediate Bits */
	uint8_t RegFrfLsb;              /* RF carrier frequency, least significant bits */
	uint8_t RegPaConfig;            /* PA selection and output power control */
	uint8_t RegPaRamp;              /* Control of PA ramp*/
	uint8_t RegOcp;                 /* Over Current Protection control */
	uint8_t RegLna;                 /* LNA settings */
	uint8_t RegRxConfig;            /* AFC, AGC, ctrl */
	uint8_t RegRssiConfig;          /* RSSI-related settings */
	uint8_t RegRssiCollision;       /* RSSI collision detector */
	uint8_t RegRssiThresh;          /* RSSI Threshold control */
	uint8_t RegRssiValue;           /* RSSI value in dBm */
	uint8_t RegRxBw;                /* Channel Filter BW Control */
	uint8_t RegAfcBw;               /* Channel Filter BW control during the AFC routine */
	uint8_t RegOokPeak;             /* OOK demodulator selection and control in peak mode */
	uint8_t RegOokFix;              /* Fixed threshold control of the OOK demodulator */
	uint8_t RegOokAvg;              /* Average threshold control of the OOK demodulator */
	uint8_t Reserved17;              /* Average threshold control of the OOK demodulator */
	uint8_t Reserved18;              /* Average threshold control of the OOK demodulator */
	uint8_t Reserved19;              /* Average threshold control of the OOK demodulator */
	uint8_t RegAfcFei;              /* AFC and FEI control and status */
	uint8_t RegAfcMsb;              /* MSB of the frequency correction of the AFC */
	uint8_t RegAfcLsb;              /* LSB of the frequency correction of the AFC */
	uint8_t RegFeiMsb;              /* MSB of the calculated frequency error */
	uint8_t RegFeiLsb;              /* LSB of the calculated frequency error */
	uint8_t RegPreambleDetect;      /* Settings of preamble detector */
	uint8_t RegRxTimeout1;          /* Timeout duration between RX request and RSSI detection */
	uint8_t RegRxTimeout2;          /* Timeout duration between RSSI detection and PayloadReady */
	uint8_t RegRxTimeout3;          /* Timeout duration between RX request and RSSI detection */
	uint8_t RegRxDelay;             /* Delay between Rx cycles */
	uint8_t RegOsc;                 /* RC oscillator settings, CLKOUT frequency */
	uint8_t RegPreambleMsb;         /* Preamble length, MSB */
	uint8_t RegPreambleLsb;         /* Preamble length, LSB */
	uint8_t RegSyncConfig;          /* Sync Word Recognition control */
	uint8_t RegSyncValue1;          /* Sync Word byte 1 */
	uint8_t RegSyncValue2;          /* Sync Word byte 2 */
	uint8_t RegSyncValue3;          /* Sync Word byte 3 */
	uint8_t RegSyncValue4;          /* Sync Word byte 4 */
	uint8_t RegSyncValue5;          /* Sync Word byte 5 */
	uint8_t RegSyncValue6;          /* Sync Word byte 6 */
	uint8_t RegSyncValue7;          /* Sync Word byte 7 */
	uint8_t RegSyncValue8;          /* Sync Word byte 8 */
	uint8_t RegPacketConfig1;       /* Packet mode settings */
	uint8_t RegPacketConfig2;       /* Packet mode settings */
	uint8_t RegPayloadLength;       /* Payload length setting */
	uint8_t RegNodeAdrs;            /* Node address */
	uint8_t RegBroadcastAdrs;       /* Broadcast address */
	uint8_t RegFifoThresh;          /* Fifo threshold, TX start condition */
	uint8_t RegSeqConfig1;
	uint8_t RegSeqConfig2;
	uint8_t RegTimerResol;
	uint8_t RegTimer1Coef;
	uint8_t RegTimer2Coef;
	uint8_t RegImageCal;
	uint8_t RegTemp;
	uint8_t RegLowBat;
	uint8_t RegIrqFlags1;           /* Status register: PLL Lock state, Timeout, RSSI > Threshold... */
	uint8_t RegIrqFlags2;           /* Status register: FIFO handling flags, Low battery detection... */
	uint8_t RegDioMapping1;         /* Mapping of pins DIO0 to DIO3 */
	uint8_t RegDioMapping2;         /* Mapping of pins DIO4 and DIO5/Clkout frequency */
	uint8_t RegVersion;             /* RF ID relating the silicon revision */
	uint8_t RegPllHop;              /* Control the fast frequency hopping module */
	uint8_t RegTcxo;                /* TCXO or XTAL input setting */
	uint8_t RegPaDac;               /* Higher power settings of the PA */
	uint8_t RegFormerTemp;          /* Stored temp during the former IQ Calibration */
	uint8_t RegBitRateFrac;         /* Fractional part in the bitrate division ratio */
	uint8_t RegAgcRef;              /* Adjustment of the AGC thresholds */
	uint8_t RegAgcThresh1;          /************************************/
	uint8_t RegAgcThresh2;          /************************************/
	uint8_t RegAgcThresh3;          /************************************/
	uint8_t RegPllLf;                 /* Control of the PLL bandwidth */

};

/*
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
	X(RegPllLf)

*/


typedef struct _CONFIG_SX1236
{
	// Constants
	uint32_t    Fxosc;
	double      Fstep;
	uint32_t    carrier_freq;
	uint32_t    freq_dev_hz;
	uint32_t    bitrate;
	struct      SX1236 sx1236_state;
} config_sx1236;

extern struct SX1236 regaddrs;
extern struct SX1236 POR_defaults;
extern struct CONFIG_SX1236 config_rx;

void sx1236_print_regs(SPIDriver * spip) ;

//define sx1236 packet structure
// todo: Investigate attribute__(PACKED)__? 
typedef struct _sx1236_packet
{
	uint8_t PacType;                /* Packet Type */
	uint8_t PacSequence;            /* Packet Sequence */
	uint8_t PacSourceAddress;       /* Packet Source Address */
	uint8_t PacDestAddress;         /* Packet Destination Address */
	uint8_t PacInstruction;        /* Instruction contained in packet */
	uint8_t PacData[27];            /* Packet data/response */
} sx1236_packet;

//define packet content size
#define PacketContentSize	((int)(27))

//create a structure of raw packet data so that it can be passed around functions
//C doesn't seem to handle passing array properly without fancy malloc funtions
typedef struct _sx1236_raw_packet
{
	uint8_t RawPacData[32];         /* Packet data/response */
} sx1236_raw_packet;

#define     MAX_SX_BUFF            2056

extern uint8_t sx_txbuff[MAX_SX_BUFF];
extern uint8_t sx_rxbuff[MAX_SX_BUFF];

#define Mode_RX          (1 << 4)
#define Mode_TX          (3 << 2)

#define PA0              (1 << 7)
#define PA1              (1 << 6)
#define PA2              (1 << 5)

#define PAOutputCfg(pa, power) (((pa) & (PA0 | PA1 | PA2)) | ((power) & 0x1F))

// RegIrqFlags1
#define ModeReady        (1 << 7)
#define RxReady          (1 << 6)
#define TxReady          (1 << 5)
#define PllLock          (1 << 4)
#define Rssi             (1 << 3)
#define Timeout          (1 << 2)
#define AutoMode         (1 << 1)
#define SyncAddressMatch (1 << 0)
// RegIrqFlags2
#define FifoFull         (1 << 7)
#define FifoNotEmpty     (1 << 6)
#define FifoLevel        (1 << 5)
#define FifoOverrunl     (1 << 4)
#define PacketSent       (1 << 3)
#define PayloadReady     (1 << 2)
#define CrcOk            (1 << 1)
#define LowBat           (1 << 0)

//define packet contents
//packet type
#define InstructionPacket	((uint8_t)(0x01))
#define DataPacket			((uint8_t)(0x02))
#define AckPacket			((uint8_t)(0x03))
//address for packets
#define BoradcastAddress	((uint8_t)(0x00))
#define GroundStation1		((uint8_t)(0x01))
#define GroundStation2		((uint8_t)(0x02))
#define GroundStation3		((uint8_t)(0x03))
#define Oresat1				((uint8_t)(0x10))
#define Oresat2				((uint8_t)(0x20))
#define Oresat3				((uint8_t)(0x30))


void sx1236_print_regs(SPIDriver * spip) ;
void sx1236_init_state(struct SX1236 * s) ;
void sx1236_reset(void) ;

void sx1236_read(SPIDriver * spip, uint8_t address, uint8_t * rx_buf, uint8_t n);
uint8_t sx1236_read_reg(SPIDriver * spip, uint8_t address) ;
void sx1236_write(SPIDriver * spip, uint8_t address, uint8_t * tx_buf, uint8_t n);
void sx1236_write_reg(SPIDriver * spip, uint8_t address, uint8_t newval);
void sx1236_check_reg(SPIDriver * spip, uint8_t address, uint8_t checkval);
void sx1236_write_FIFO(SPIDriver * spip, uint8_t value);
uint8_t sx1236_read_FIFO(SPIDriver * spip);

void sx1236_configure(SPIDriver * spip, config_sx1236 * c);

void sx1236_write_carrier_freq(SPIDriver * spip, config_sx1236 * c);
void sx1236_set_freq_deviation(SPIDriver * spip, config_sx1236 * c);
void sx1236_set_bitrate(SPIDriver * spip, config_sx1236 * c);

void sx1236_packet_tx(SPIDriver * spip, sx1236_packet p);
void sx1236_packet_rx(SPIDriver * spip, config_sx1236 * c, sx1236_raw_packet * r);
void sx1236_packet_rx2(SPIDriver * spip, config_sx1236 * c, sx1236_raw_packet * r);
void sx1236_packet_format(sx1236_packet * p, sx1236_raw_packet * r);
void sx1236_create_data_packet_tx(SPIDriver * spip, uint8_t data[], int data_size);
void sx1236_create_instruction_packet_tx(SPIDriver * spip, uint8_t inst);

#endif
//! @}
