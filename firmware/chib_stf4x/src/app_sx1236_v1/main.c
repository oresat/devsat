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

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

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
	uint8_t regval = 0;

	regval = sx1236_read_reg(&SPID1, regaddrs.RegVersion);
	chprintf(DEBUG_CHP, "1:ID, 0x%x:\t0x%x\r\n", regaddrs.RegVersion, regval);
	sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x12);

	uint8_t nv = 0x1b;
	sx1236_write_reg(&SPID1, regaddrs.RegBitrateMsb, nv);
	sx1236_check_reg(&SPID1, regaddrs.RegBitrateMsb, nv);

	// chThdSleepMilliseconds(1000);

	// chThdSleepMilliseconds(300) ;
	// sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x13, &receivedval);

	// if(sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x13, &receivedval)) {
	// chprintf(DEBUG_CHP, "check_reg error***: reg[0x%x] should be: 0x%x, got: 0x%x", regaddrs.RegVersion, 0x12, receivedval);
	// };

	while (true)
	{
		chThdSleepMilliseconds(500);
		palTogglePad(GPIOA, GPIOA_SX_TESTOUT);
		chprintf(DEBUG_CHP, ".");
		chThdSleepMilliseconds(500);
		palTogglePad(GPIOA, GPIOA_SX_TESTOUT);
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



