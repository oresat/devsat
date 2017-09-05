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
#include "board.h"

#include "util_general.h"
#include "util_version.h"
#include "util_numbers.h"

#include "sx1236.h"

#define     DEBUG_SERIAL        SD2
#define     DEBUG_CHP           ((BaseSequentialStream *) &DEBUG_SERIAL)

#define     F_XOSC              (32000000U)
#define     F_STEP              ((double)(61.03515625)) //  (Fxosc/2^9)
#define     APP_CARRIER_FREQ    (436500000U)
#define     APP_FREQ_DEV        (2500U)
#define     APP_BITRATE         (2400U)

// RSSI Thresh
#define     RSSI_THRESH         ((uint8_t)(0x70U))


// Sync bytes
#define     SX1236_SYNCVALUE1   ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE2   ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE3   ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE4   ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE5   ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE6   ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE7   ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE8   ((uint8_t) (0xe7U))

// Payload length
#define     PAYLOAD_LENGTH      ((uint8_t) (0x05))
#define     FIFO_THRESH         ((uint8_t) (0x05))

// #define  F_STEP  (Fxosc/2^9)

struct CONFIG_SX1236_RX config_rx =
{
	.Fxosc              = F_XOSC,
	.Fstep              = F_STEP,
	.carrier_freq       = APP_CARRIER_FREQ,
	.freq_dev_hz        = APP_FREQ_DEV,
	.bitrate            = APP_BITRATE,

	.RegOpMode          = SX1236_LOW_FREQ_MODE | SX1236_FSK_MODE | SX1236_STANDBY_MODE,
	.RegPaRamp          = SX1236_NO_SHAPING,
	.RegPacketConfig1   = SX1236_VARIABLE_PACKET,
	.RegPacketConfig2   = SX1236_PACKET_MODE,
	.RegPllLf           = SX1236_PLLBW_75KHZ,
	.RegRssiThresh      = RSSI_THRESH,
	.RegSyncConfig      = SX1236_SYNC_ON,
	.RegSyncValue1      = SX1236_SYNCVALUE1,
	.RegSyncValue2      = SX1236_SYNCVALUE2,
	.RegSyncValue3      = SX1236_SYNCVALUE3,
	.RegSyncValue4      = SX1236_SYNCVALUE4,
	.RegSyncValue5      = SX1236_SYNCVALUE5,
	.RegSyncValue6      = SX1236_SYNCVALUE6,
	.RegSyncValue7      = SX1236_SYNCVALUE7,
	.RegSyncValue8      = SX1236_SYNCVALUE8,

	.RegPayloadLength   = PAYLOAD_LENGTH,
	.RegFifoThresh      = FIFO_THRESH,
	.RegRxConfig        = SX1236_AFC_AUTO_ON,
	.RegAfcFei          = SX1236_AFC_AUTO_CLEAR_ON,
};

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

	chprintf(DEBUG_CHP, "Reset\r\n");
	sx1236_reset() ;
}

void main_loop(void)
{
	sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x12);

	sx1236_configure_rx(&SPID1, &config_rx);

	while (true)
	{
		chThdSleepMilliseconds(500);
		palTogglePad(GPIOA, GPIOA_SX_TESTOUT);
		chprintf(DEBUG_CHP, ".");
	}
}

int main(void)
{
	halInit();
	chSysInit();
	app_init();

	main_loop();
	return 0;
}



