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
#include "util_numbers.h"

#include "ltc2990.h"
#include "solar_v1.h"

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

static const I2CConfig i2cfg1 =
{
	I2C_100KHZ_TIMINGR,
	0,
	0,
};

/* I2C */
const  uint8_t           LTC2990_I2C_ADDR   =    0x98;

static ltc2990_data      monitor_data;

inline void i2c_report_error(i2cflags_t i2c_errors)
{
	if(i2c_errors != 0)
	{
		chprintf(DEBUG_CHP, "I2C_ERROR!: %d\r\n", i2c_errors);
	}
}

static void demo_measure(void)
{
	i2cflags_t           i2c_errors  = 0x0;
	volatile uint8_t     regval      = 0xf;

	regval = 0xf;

	/* CONTROL Setup */
	regval = LTC2990_CONTROL_ACQ_SINGLE
	         | LTC2990_CONTROL_ALL_MODE_4_3
	         | LTC2990_CONTROL_MODE_1_2_0;
	ltc2990_writereg(LTC2990_CONTROL, regval, &i2c_errors);
	i2c_report_error(i2c_errors);

	while(1)
	{
		/* TRIGGER */
		regval = 0xf;
		chprintf(DEBUG_CHP, "\t***Trigger***\r\n");
		ltc2990_writereg(LTC2990_TRIGGER, regval, &i2c_errors);
		i2c_report_error(i2c_errors);
		chThdSleepMilliseconds(LTC2990_TRIGGER_WAIT_MS);

		regval = ltc2990_readreg(LTC2990_CONTROL, &i2c_errors);
		i2c_report_error(i2c_errors);

		/* CONTROL check status */
		if(!ltc2990_conversion_done(regval))
		{
			chprintf(DEBUG_CHP, "LTC2990 Error: Conversion not finished\r\n");
		}
		else
		{
			signed int tint = 0;
			signed int vcc  = 0;
			/* READ ALL */
			ltc2990_read_all(&monitor_data, &i2c_errors);
			i2c_report_error(i2c_errors);

			/* TINT */
			ltc2990_error derror;
			tint = ltc2990_calc_tint(&monitor_data, &derror);
			if(derror == LTC2990_OK)
			{
				chprintf(DEBUG_CHP, "TINT is %iC\r\n", tint );
			}
			else
			{
				chprintf(DEBUG_CHP, "TINT ERROR: %d\r\n", derror);
			}

			/* VCC */
			vcc = ltc2990_calc_vcc(&monitor_data, &derror );
			if(derror == LTC2990_OK)
			{
				chprintf(DEBUG_CHP, "VCC is %d mV\r\n", vcc);
			}
			else
			{
				chprintf(DEBUG_CHP, "VCC ERROR: %d\r\n", derror);
			}

			/* Current */
			signed int current = 0;
			current = solar_v1_calc_current(&monitor_data, &derror);
			if(derror == LTC2990_OK)
			{
				chprintf(DEBUG_CHP, "Current is %d mA\r\n", current);
			}
			else
			{
				chprintf(DEBUG_CHP, "Current ERROR: %d\r\n", derror);
			}

			/* External Temp */
			signed int text = 0;
			chprintf(DEBUG_CHP, "V3_MSB: 0x%x\r\nV3_LSB: 0x%x\r\n", monitor_data.V3_MSB, monitor_data.V3_LSB);
			text    = solar_v1_calc_temp(&monitor_data, &derror) ;
			if(derror == LTC2990_OK)
			{
				chprintf(DEBUG_CHP, "External T is %d C\r\n", text);
			}
			else
			{
				chprintf(DEBUG_CHP, "External T ERROR: %d\r\n", derror);
			}
		}
		chprintf(DEBUG_CHP, "\r\n**********\r\n");
		chThdSleepMilliseconds(4000);
	}
}

/* APP */
static void app_init(void)
{
	// start up debug output, chprintf(DEBUG_CHP,...)
	sdStart(&DEBUG_SERIAL, &ser_cfg);

	chprintf(DEBUG_CHP, "\r\n");
	i2cInit();
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
	chprintf(DEBUG_CHP, "\r\n**********\r\n");
	chprintf(DEBUG_CHP, "Demo\r\n");
	demo_measure();

	while (true)
	{
		chprintf(DEBUG_CHP, ".");
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

