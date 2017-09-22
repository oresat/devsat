/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

// #include "util_general.h"
#include "util_version.h"
#include "util_numbers.h"
#include "board.h"

#include "dio_ext.h"
#include "sx1236.h"

EVENTSOURCE_DECL(DIO0_EVT);
EVENTSOURCE_DECL(DIO1_EVT);
EVENTSOURCE_DECL(DIO2_EVT);
EVENTSOURCE_DECL(DIO3_EVT);
EVENTSOURCE_DECL(DIO4_EVT);
EVENTSOURCE_DECL(DIO5_EVT);

#define     DEBUG_SERIAL                    SD2
#define     DEBUG_CHP                       ((BaseSequentialStream *) &DEBUG_SERIAL)

#define     F_XOSC                          (32000000U)
#define     F_STEP                          ((double)(61.03515625)) //  (Fxosc/2^9)
#define     APP_CARRIER_FREQ                (436500000U)
#define     APP_FREQ_DEV                    (2500U)

// #define     APP_BITRATE                     (2400U)
#define     APP_BITRATE                     (38400U)

// RSSI Thresh
#define     RSSI_THRESH                     ((uint8_t)(0x70U))

// SeqConfig1
#define     FromTransmit_RX                 ((uint8_t)(0b1<<0))
#define     FromIdle_RX                     ((uint8_t)(0b1<<1))
#define     LowPowerSelect_IDLE             ((uint8_t)(0b1<<2))
#define     FromStart_TO_LP                 ((uint8_t)(0b00<<3))
#define     FromStart_TO_RX                 ((uint8_t)(0b01<<3))
#define     FromStart_TO_TX                 ((uint8_t)(0b10<<3))
#define     FromStart_TO_TX_FIFOINT         ((uint8_t)(0b11<<3))
#define     Idle_TO_STANDBY                 ((uint8_t)(0b00<<5))
#define     SEQ_STOP                        ((uint8_t)(0b1<<6))
#define     SEQ_START                       ((uint8_t)(0b1<<7))

// SeqConfig1
#define     FromRX_PKT_RX_PLD_RDY           ((uint8_t)(0b001<<5))
#define     FromRX_PKT_RX_CRC_OK            ((uint8_t)(0b011<<5))
#define     FromRX_Timeout_TO_RX_ST         ((uint8_t)(0b00<<3))
#define     FromPKT_RXD_TO_RX               ((uint8_t)(0b100<<0))
#define     FromPKT_RXD_TO_LP_SELECT        ((uint8_t)(0b010<<0))

// Sync bytes
#define     SX1236_SYNCVALUE1               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE2               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE3               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE4               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE5               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE6               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE7               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE8               ((uint8_t) (0xe7U))

// Payload length
#define     PAYLOAD_LENGTH                  ((uint8_t) (0x05U))
#define     FIFO_THRESH                     ((uint8_t) (0x05U))

// Structure to hold configuration for test
static config_sx1236 dut_config ;

static void init_rx_packet(config_sx1236 * s)
{
	s->Fxosc                        	= F_XOSC;
	s->Fstep                        	= F_STEP;
	s->carrier_freq                 	= APP_CARRIER_FREQ;
	s->freq_dev_hz                  	= APP_FREQ_DEV;
	s->bitrate                      	= APP_BITRATE;

	sx1236_init_state(&s->sx1236_state); 

	s->sx1236_state.RegOpMode          = SX1236_LOW_FREQ_MODE | SX1236_FSK_MODE | SX1236_STANDBY_MODE;
	s->sx1236_state.RegPaRamp          = SX1236_NO_SHAPING;
	s->sx1236_state.RegDioMapping1     = SX1236_DIO0_PAYLOAD_RDY | SX1236_DIO2_RX_RDY;
	s->sx1236_state.RegDioMapping2     = SX1236_DIO4_RX_TIMEOUT;
	s->sx1236_state.RegPacketConfig1   = SX1236_VARIABLE_PACKET;
	s->sx1236_state.RegPacketConfig2   = SX1236_PACKET_MODE;
	s->sx1236_state.RegPllLf           = SX1236_PLLBW_75KHZ;
	s->sx1236_state.RegRssiThresh      = RSSI_THRESH;
	s->sx1236_state.RegSyncConfig      = SX1236_SYNC_ON;
	s->sx1236_state.RegSyncValue1      = SX1236_SYNCVALUE1;
	s->sx1236_state.RegSyncValue2      = SX1236_SYNCVALUE2;
	s->sx1236_state.RegSyncValue3      = SX1236_SYNCVALUE3;
	s->sx1236_state.RegSyncValue4      = SX1236_SYNCVALUE4;
	s->sx1236_state.RegSyncValue5      = SX1236_SYNCVALUE5;
	s->sx1236_state.RegSyncValue6      = SX1236_SYNCVALUE6;
	s->sx1236_state.RegSyncValue7      = SX1236_SYNCVALUE7;
	s->sx1236_state.RegSyncValue8      = SX1236_SYNCVALUE8;
	s->sx1236_state.RegSeqConfig1      = FromTransmit_RX | FromIdle_RX | LowPowerSelect_IDLE | FromStart_TO_RX | Idle_TO_STANDBY;
	s->sx1236_state.RegSeqConfig2      = FromRX_PKT_RX_PLD_RDY | FromRX_Timeout_TO_RX_ST | FromPKT_RXD_TO_RX;
	s->sx1236_state.RegPayloadLength   = PAYLOAD_LENGTH;
	s->sx1236_state.RegFifoThresh      = FIFO_THRESH;
	s->sx1236_state.RegRxConfig        = SX1236_AFC_AUTO_ON;
	s->sx1236_state.RegAfcFei          = SX1236_AFC_AUTO_CLEAR_ON;
}

static void init_tx_continuous(config_sx1236 * s)
{
	s->Fxosc                        	= F_XOSC;
	s->Fstep                        	= F_STEP;
	s->carrier_freq                 	= APP_CARRIER_FREQ;
	s->freq_dev_hz                  	= APP_FREQ_DEV;
	s->bitrate                      	= APP_BITRATE;

	sx1236_init_state(&s->sx1236_state); 

	s->sx1236_state.RegOpMode          = SX1236_LOW_FREQ_MODE | SX1236_FSK_MODE | SX1236_STANDBY_MODE;
	s->sx1236_state.RegPaRamp          = SX1236_NO_SHAPING;
	s->sx1236_state.RegDioMapping1     = SX1236_DIO0_PAYLOAD_RDY | SX1236_DIO2_RX_RDY;
	s->sx1236_state.RegDioMapping2     = SX1236_DIO4_RX_TIMEOUT;
	s->sx1236_state.RegPacketConfig1   = SX1236_VARIABLE_PACKET;
	s->sx1236_state.RegPacketConfig2   = SX1236_CONTINUOUS_MODE;
	s->sx1236_state.RegPllLf           = SX1236_PLLBW_75KHZ;
	s->sx1236_state.RegRssiThresh      = RSSI_THRESH;
	s->sx1236_state.RegSyncConfig      = SX1236_SYNC_ON;
	s->sx1236_state.RegSyncValue1      = SX1236_SYNCVALUE1;
	s->sx1236_state.RegSyncValue2      = SX1236_SYNCVALUE2;
	s->sx1236_state.RegSyncValue3      = SX1236_SYNCVALUE3;
	s->sx1236_state.RegSyncValue4      = SX1236_SYNCVALUE4;
	s->sx1236_state.RegSyncValue5      = SX1236_SYNCVALUE5;
	s->sx1236_state.RegSyncValue6      = SX1236_SYNCVALUE6;
	s->sx1236_state.RegSyncValue7      = SX1236_SYNCVALUE7;
	s->sx1236_state.RegSyncValue8      = SX1236_SYNCVALUE8;
	s->sx1236_state.RegSeqConfig1      = FromTransmit_RX | FromIdle_RX | LowPowerSelect_IDLE | FromStart_TO_RX | Idle_TO_STANDBY;
	s->sx1236_state.RegSeqConfig2      = FromRX_PKT_RX_PLD_RDY | FromRX_Timeout_TO_RX_ST | FromPKT_RXD_TO_RX;
	s->sx1236_state.RegPayloadLength   = PAYLOAD_LENGTH;
	s->sx1236_state.RegFifoThresh      = FIFO_THRESH;
	s->sx1236_state.RegRxConfig        = SX1236_AFC_AUTO_ON;
	s->sx1236_state.RegAfcFei          = SX1236_AFC_AUTO_CLEAR_ON;
}

static SerialConfig ser_cfg =
{
	115200,     //Baud rate
	0,          //
	0,          //
	0,          //
};

/*
 * Only one device on SPI bus: sx1236
 */
static const SPIConfig spicfg =
{
	NULL,               // Operation complete callback
	GPIOA,              // Slave select port
	GPIOA_SPI1_NSS,     // Slave select pad
	// SPI cr1 data (see 446 ref man.)
	SPI_CR1_SPE     |   // SPI enable
	SPI_CR1_MSTR    |   // Master
	//SPI_CR1_BR_2    |
	SPI_CR1_BR_1    |
	SPI_CR1_BR_0   |       // fpclk/16  approx 5Mhz? BR = 0x011
	SPI_CR1_SSM,
	0, // SPI_CR2_SSOE,
};

static void app_init(void)
{
	// Start up debug output, chprintf(DEBUG_CHP,...)
	sdStart(&DEBUG_SERIAL, &ser_cfg);

	set_util_fwversion(&version_info);
	set_util_hwversion(&version_info);
	chThdSleepS(S2ST(2));

	//Print FW/HW information
	chprintf(DEBUG_CHP, "\r\nFirmware Info\r\n");
	chprintf(DEBUG_CHP, "FW HASH: %s\r\n", version_info.firmware);
	chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x\r\n"
	         , version_info.hardware.id_high
	         , version_info.hardware.id_center
	         , version_info.hardware.id_low
	        );

	spiStart(&SPID1, &spicfg);

	// RX Packet test
	init_rx_packet(&dut_config);

	chprintf(DEBUG_CHP, "Reset\r\n");
	sx1236_reset() ;
}

static void dio0_evt_handler(eventid_t id)
{
	(void)id;
	chprintf(DEBUG_CHP, "dio0 event\r\n");
}

static void dio1_evt_handler(eventid_t id)
{
	(void)id;
	chprintf(DEBUG_CHP, "dio2 event\r\n");
}

static void dio2_evt_handler(eventid_t id)
{
	(void)id;
	chprintf(DEBUG_CHP, "dio2 event\r\n");
}


static void dio3_evt_handler(eventid_t id)
{
	(void)id;
	chprintf(DEBUG_CHP, "dio3 event\r\n");
}

static void dio4_evt_handler(eventid_t id)
{
	(void)id;
	chprintf(DEBUG_CHP, "dio4 event\r\n");
}

static void dio5_evt_handler(eventid_t id)
{
	(void)id;
	chprintf(DEBUG_CHP, "dio5 event\r\n");
}

static THD_WORKING_AREA(waThread_sx1236_dio, 512);
static THD_FUNCTION(Thread_sx1236_dio, arg)
{
	(void) arg;
	static const evhandler_t evhndl_sx1236_dio[] =
	{
		dio0_evt_handler,
		dio1_evt_handler,
		dio2_evt_handler,
		dio3_evt_handler,
		dio4_evt_handler,
		dio5_evt_handler
	};

	event_listener_t evl_dio0, evl_dio1, evl_dio2, evl_dio3, evl_dio4, evl_dio5;

	chRegSetThreadName("sx1236_dio");

	chEvtRegister(&DIO0_EVT,           &evl_dio0,         0);
	chEvtRegister(&DIO1_EVT,           &evl_dio1,         1);
	chEvtRegister(&DIO2_EVT,           &evl_dio2,         2);
	chEvtRegister(&DIO3_EVT,           &evl_dio3,         3);
	chEvtRegister(&DIO4_EVT,           &evl_dio4,         4);
	chEvtRegister(&DIO5_EVT,           &evl_dio5,         5);

	chprintf(DEBUG_CHP, "Thread started: %s\r\n", "sx1236_dio");
	while (TRUE)
	{
		chEvtDispatch(evhndl_sx1236_dio, chEvtWaitOneTimeout(EVENT_MASK(0), MS2ST(50)));
	}
}

static void start_threads(void)
{
	chThdCreateStatic(waThread_sx1236_dio,      sizeof(waThread_sx1236_dio),   NORMALPRIO, Thread_sx1236_dio, NULL);
}


static void main_loop(void)
{
	chThdSleepMilliseconds(500);
	chprintf(DEBUG_CHP, "\r\n");
	sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x12);

	sx1236_configure(&SPID1, &dut_config);

	while (true)
	{
		chThdSleepMilliseconds(500);
		palTogglePad(GPIOA, GPIOA_SX_TESTOUT);
		chprintf(DEBUG_CHP, ".");
		// Start/restart the sequencer
		sx1236_write_reg(&SPID1, regaddrs.RegSeqConfig1, dut_config.sx1236_state.RegSeqConfig1 | SEQ_START);
		sx1236_check_reg(&SPID1, regaddrs.RegSeqConfig1, dut_config.sx1236_state.RegSeqConfig1);
		// sx1236_check_reg(&SPID1, regaddrs.RegSeqConfig1, config_rx.RegSeqConfig1 );
	}
}

int main(void)
{
	halInit();
	chSysInit();
	app_init();

	start_threads();
	main_loop();
	return 0;
}



