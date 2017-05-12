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
	I2C_400KHZ_TIMINGR,
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


static bool ltc2990_test(void)
{
	// bool firsttime = true;
	i2cflags_t  i2c_errors  = 0x0;
	volatile uint8_t     regval      = 0xf;
	while(1)
	{
		regval = 0xf;
		chprintf(DEBUG_CHP, "I2C Read addr:\t\t0x%x\r\n", LTC2990_I2C_ADDR);

		/* CONTROL */
		regval = LTC2990_CONTROL_ACQ_SINGLE
		         | LTC2990_CONTROL_ALL_MODE_4_3
		         | LTC2990_CONTROL_MODE_1_2_0;

		chprintf(DEBUG_CHP, "Constructed regval is 0x%x\r\n", regval);
		ltc2990_writereg(LTC2990_CONTROL, regval, &i2c_errors);
		i2c_report_error(i2c_errors);

		regval = 0xf;
		regval = ltc2990_readreg(LTC2990_CONTROL, &i2c_errors);
		i2c_report_error(i2c_errors);
		if(i2c_errors == 0)
		{
			chprintf(DEBUG_CHP, "Control regval is\t0x%x\r\n", regval);
		}

		/* TRIGGER */
		regval = 0xf;
		chprintf(DEBUG_CHP, "\t***Trigger***\r\n");
		ltc2990_writereg(LTC2990_TRIGGER, regval, &i2c_errors);
		i2c_report_error(i2c_errors);
		chThdSleepMilliseconds(500);

		/*
		 * [> STATUS <]
		 * regval = 0xf;
		 * regval = ltc2990_readreg(LTC2990_STATUS, &i2c_errors);
		 * i2c_report_error(i2c_errors);
		 * if(i2c_errors == 0)
		 * {
		 *     chprintf(DEBUG_CHP, "Status regval is\t0x%x\r\n", regval);
		 * }
		 */

		// [> STATUS <]
		// regval = 0xf;
		// regval = ltc2990_readreg(LTC2990_STATUS, &i2c_errors);
		// i2c_report_error(i2c_errors);
		// if(i2c_errors == 0)
		// {
		// chprintf(DEBUG_CHP, "Status regval is\t0x%x\r\n", regval);
		// }

		uint8_t tint_msb = 0;
		/* TINT_MSB */
		tint_msb = ltc2990_readreg(LTC2990_TINT_MSB, &i2c_errors);
		i2c_report_error(i2c_errors);
		if(i2c_errors == 0)
		{
		chprintf(DEBUG_CHP, "TINT_MSB regval is\t0x%x\r\n", tint_msb);
		}

		uint8_t tint_lsb = 0;
		/* TINT_LSB */
		tint_lsb = ltc2990_readreg(LTC2990_TINT_LSB, &i2c_errors);
		i2c_report_error(i2c_errors);
		if(i2c_errors == 0)
		{
		chprintf(DEBUG_CHP, "TINT_LSB regval is\t0x%x\r\n", tint_lsb);
		}

		signed int tint = 0;
		if((0x80 & tint_msb) != 0)
		{
		//clear dv bit
		tint_msb = ((~(1 << 7)) & tint_msb);
		tint = sign_extend_12bit((tint_msb << 8) | tint_lsb);
		chprintf(DEBUG_CHP, "TINT is %iC\r\n", tint / 16);
		}

		/* TRIGGER */
		regval = 0xf;
		chprintf(DEBUG_CHP, "\t***Trigger***\r\n");
		ltc2990_writereg(LTC2990_TRIGGER, regval, &i2c_errors);
		i2c_report_error(i2c_errors);
		chThdSleepMilliseconds(500);

		/* READ ALL */
		ltc2990_read_all(&monitor_data, &i2c_errors);
		if(i2c_errors == 0)
		{
			chprintf(DEBUG_CHP, "Status regval is\t0x%x\r\n", monitor_data.STATUS);
			chprintf(DEBUG_CHP, "Control regval is\t0x%x\r\n", monitor_data.CONTROL);
			chprintf(DEBUG_CHP, "Trigger regval is\t0x%x\r\n", monitor_data.TRIGGER);
			chprintf(DEBUG_CHP, "NA regval is\t0x%x\r\n", monitor_data.NA);
			chprintf(DEBUG_CHP, "T_INT_MSB regval is\t0x%x\r\n", monitor_data.T_INT_MSB);
			chprintf(DEBUG_CHP, "T_INT_LSB regval is\t0x%x\r\n", monitor_data.T_INT_LSB);
			chprintf(DEBUG_CHP, "V1_MSB regval is\t0x%x\r\n", monitor_data.V1_MSB);
			chprintf(DEBUG_CHP, "V1_LSB regval is\t0x%x\r\n", monitor_data.V1_LSB);
			chprintf(DEBUG_CHP, "V2_MSB regval is\t0x%x\r\n", monitor_data.V2_MSB);
			chprintf(DEBUG_CHP, "V2_LSB regval is\t0x%x\r\n", monitor_data.V2_LSB);
			chprintf(DEBUG_CHP, "V3_MSB regval is\t0x%x\r\n", monitor_data.V3_MSB);
			chprintf(DEBUG_CHP, "V3_LSB regval is\t0x%x\r\n", monitor_data.V3_LSB);
			chprintf(DEBUG_CHP, "V4_MSB regval is\t0x%x\r\n", monitor_data.V4_MSB);
			chprintf(DEBUG_CHP, "V4_LSB regval is\t0x%x\r\n", monitor_data.V4_LSB);
			chprintf(DEBUG_CHP, "VCC_MSB regval is\t0x%x\r\n", monitor_data.VCC_MSB);
			chprintf(DEBUG_CHP, "VCC_LSB regval is\t0x%x\r\n", monitor_data.VCC_LSB);
		}
		else
		{
			chprintf(DEBUG_CHP, "*** I2C_ERROR: 0x%x\r\n", i2c_errors);
		}

		chprintf(DEBUG_CHP, "*************************\r\n\r\n");
		chThdSleepMilliseconds(4500);
	}
	return true;
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
	ltc2990_test();

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

