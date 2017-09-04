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
#define     APP_CARRIER_FREQ    436500000U
#define     APP_FREQ_DEV        2500U
#define     APP_BITRATE         2400U

// #define  F_STEP  (Fxosc/2^9)

struct CONFIG_SX1236_RX config_rx =
{
	.Fxosc              = F_XOSC,
	.Fstep              = F_STEP,
	.carrier_freq       = APP_CARRIER_FREQ,
	.freq_dev_hz        = APP_FREQ_DEV,
	.bitrate         = APP_BITRATE,

	.RegOpMode          = SX1236_FS_MODE,

	// .RegFifo;
};

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
	uint32_t    rate      = 0;

	uint8_t     bitrate_msb  = 0;
	uint8_t     bitrate_lsb  = 0;

	rate          = (uint32_t)incr_rnd((fxosc / bitrate), 0.1);

	bitrate_msb      = (rate >> 8) & 0x3f;
	bitrate_lsb      = rate        & 0xff;

	sx_txbuff[0] 	 = bitrate_msb;
	sx_txbuff[1] 	 = bitrate_lsb;

	sx1236_write(spip, regaddrs.RegBitrateMsb, sx_txbuff, 2);
	// sx1236_check_reg(spip, regaddrs.RegBitrateLsb, bitrate_lsb);
	// sx1236_check_reg(spip, regaddrs.RegBitrateMsb, bitrate_msb);
}

void sx1236_configure_rx(SPIDriver * spip, struct CONFIG_SX1236_RX * c)
{
	sx1236_write_reg(spip, regaddrs.RegOpMode, c->RegOpMode);
	sx1236_write_carrier_freq(spip, c->carrier_freq, c->Fstep);
	sx1236_set_freq_deviation(spip, c->freq_dev_hz, c->Fstep );
	sx1236_set_bitrate(spip, c->Fxosc, c->bitrate); 



	sx1236_check_reg(spip, regaddrs.RegOpMode, c->RegOpMode);
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

	chprintf(DEBUG_CHP, "Reset\r\n");
	sx1236_reset() ;
}

void main_loop(void)
{
	sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x12);

	sx1236_configure_rx(&SPID1, &config_rx);


	// chprintf(DEBUG_CHP, "1:ID, 0x%x:\t0x%x\r\n", regaddrs.RegVersion, regval);
	// uint8_t nv = 0x1b;
	// sx1236_write_reg(&SPID1, regaddrs.RegBitrateMsb, nv);
	// sx1236_check_reg(&SPID1, regaddrs.RegBitrateMsb, nv);

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



