/*! \file main.c
 *  app_solardemo
 *
 * Serial terminal setting are in SerialConfig structure.
 */

/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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

/*!
 * \defgroup main app_solar main
 *
 * @{
 */
#include <stdbool.h>
#include "ch.h"

#include "board.h"
#include "hal.h"
#include "chprintf.h"

#include "util_general.h"
#include "util_version.h"

#include "ltc2990.h"

#define APP_NAME                "solardemo"

static SerialConfig ser_cfg =
{
	115200,
	0,
	0,
	0,
};

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

/* HSI Clock selected 8Mhz - see mcuconf.h Wed 03 May 2017 12:54:49 (PDT) */
/* Ref: 26.4.10 table in Reference Manual for stm32f0 */
typedef enum
{
	I2C_10KHZ_TIMINGR  = 0x1042C3C7,
	I2C_400KHZ_TIMINGR = 0x00310309,
} solar_i2c_cfg;

static const I2CConfig i2cfg1 =
{
	I2C_10KHZ_TIMINGR,
	0,
	0,
};

/* I2C */
static uint8_t    i2c_rxbuf[LTC2990_I2C_TX_BUFSIZE];
static uint8_t    i2c_txbuf[LTC2990_I2C_RX_BUFSIZE];
const  uint8_t    LTC2990_I2C_ADDR   =    0x98;
static i2cflags_t i2c_errors         =    0;

/* TODO: To be reviewed, too STM32-centric.*/
/**
 * @name    I2C bus error conditions
 * @{
 */
// #define I2C_NO_ERROR               0x00    [>*< @brief No error.            <]
// #define I2C_BUS_ERROR              0x01    [>*< @brief Bus Error.           <]
// #define I2C_ARBITRATION_LOST       0x02    [>*< @brief Arbitration Lost.    <]
// #define I2C_ACK_FAILURE            0x04    [>*< @brief Acknowledge Failure. <]
// #define I2C_OVERRUN                0x08    [>*< @brief Overrun/Underrun.    <]
// #define I2C_PEC_ERROR              0x10    [>*< @brief PEC Error in
// reception.                  */
// #define I2C_TIMEOUT                0x20    [>*< @brief Hardware timeout.    <]
// #define I2C_SMB_ALERT              0x40    [>*< @brief SMBus Alert.         <]


static bool ltc2990_init(void)
{
	msg_t status   = MSG_OK;
	systime_t tmo  = MS2ST(4);


	while(1)
	{
		status = MSG_OK;
		i2c_rxbuf[0] = 0xff;
		chprintf(DEBUG_CHP, "Read addr:\t\t0x%x\r\n", LTC2990_I2C_ADDR);
		chprintf(DEBUG_CHP, "Write addr:\t\t0x%x\r\n", I2C_WRITEADDR(LTC2990_I2C_ADDR));

		i2c_txbuf[0] = LTC2990_STATUS; /* register address */
		i2cAcquireBus(&I2CD1);
		status       = i2cMasterTransmitTimeout(&I2CD1, LTC2990_I2C_ADDR, i2c_txbuf, 1, i2c_rxbuf, 1, tmo);
		i2cReleaseBus(&I2CD1);

		chprintf(DEBUG_CHP, "Read from STATUS:\t\t0x%x\r\n", i2c_rxbuf[0]);


		if (status != MSG_OK)
		{
			i2c_errors = i2cGetErrors(&I2CD1);
			chprintf(DEBUG_CHP, "I2C_ERROR!: %d\r\n", i2c_errors);
		}
		chThdSleepMilliseconds(1500);
	}

	// i2c_txbuf[0] = I2C_WRITEADDR(LTC2990_TRIGGER); [> register address <]
	// i2c_txbuf[1] = 0xff;
	// i2c_rxbuf[0] = 0xff;
	// i2cAcquireBus(&I2CD1);
	// status       = i2cMasterTransmitTimeout(&I2CD1, LTC2990_I2C_ADDR, i2c_txbuf, 2, i2c_rxbuf, 0, tmo);
	// i2cReleaseBus(&I2CD1);

	// if (status != MSG_OK)
	// {
	// i2c_errors = i2cGetErrors(&I2CD1);
	// chprintf(DEBUG_CHP, "I2C_ERROR!: %d\r\n", i2c_errors);
	// }

	return true;
}


/* APP */
static void app_init(void)
{
	// start up debug output, chprintf(DEBUG_CHP,...)
	sdStart(&DEBUG_SERIAL, &ser_cfg);

	chprintf(DEBUG_CHP, "\r\n");
	i2cStart(&I2CD1, &i2cfg1);
	chprintf(DEBUG_CHP, "-------\r\n\r\n");
	chprintf(DEBUG_CHP, "I2C1 Initialized.\r\n");

	chprintf(DEBUG_CHP, "-------\r\n\r\n");
	set_util_fwversion(&version_info);
	set_util_hwversion(&version_info);

	chprintf(DEBUG_CHP, "FW HASH: %s\r\n", version_info.firmware);
	chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x\r\n"
	         , version_info.hardware.id_high
	         , version_info.hardware.id_center
	         , version_info.hardware.id_low
	        );
	chprintf(DEBUG_CHP, "-------\r\n\r\n");
}

/*! \brief main application loop
 */
static void main_app(void)
{
	app_init();
	chprintf(DEBUG_CHP, "app_%s started.\r\n", APP_NAME);
	ltc2990_init();

	while (true)
	{
		// uint32_t ltc_id;
		chprintf(DEBUG_CHP, ".");
		// ltc_id = ltc2990_id();
		chThdSleepMilliseconds(1000);
	}
}

int main(void)
{
	halInit();
	chSysInit();

	main_app();

	return(0);
}

//! @}

