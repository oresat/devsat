/*! \file main.c
 *  app_solardemo
 *
 */

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

/*!
 * \defgroup main app_solardemo main
 *
 * @{
 */
#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "board.h"

#include "util_general.h"
#include "util_version.h"
#include "util_numbers.h"

#include "ltc2990.h"
#include "solar_v1.h"

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

//Variable declarations
static ltc2990_data      monitor_data;
static solar_v1_p        params;

/*
 * Serial configuration
 */
static SerialConfig ser_cfg =
{
    9600,
    0,
    0,
    0,
};


/*
 * I2C configuration
 */
static const I2CConfig i2cfg1 =
{
    I2C_100KHZ_TIMINGR,
    0,
    0,
};
const  uint8_t           LTC2990_I2C_ADDR   =    0x98;

/*
 * CAN Register configuration
 * See section 22.7.7 on the STM32 reference manual.
 * Timing calculator:
 * http://www.bittiming.can-wiki.info/
 */
static const CANConfig cancfg = {
    // MCR (Master Control Register)
    CAN_MCR_ABOM      |     //Automatic Bus-Off Management
    CAN_MCR_AWUM      |     //Automatic Wakeup Mode
    CAN_MCR_TXFP      ,     //Transmit FIFO Priority
    // BTR (Bit Timing Register)
    // Note: Convert to zero based values here when using the calculator
    // CAN_BTR_LBKM     |     //Loopback Mode (Debug)
    CAN_BTR_SJW(0)    |     //Synchronization Jump Width
    CAN_BTR_TS1(12)   |     //Time Segment 1
    CAN_BTR_TS2(1)    |     //Time Segment 2
    CAN_BTR_BRP(5)          //Bit Rate Prescaler
};

inline static void lcd_clear(void)
{
    streamPut(DEBUG_CHP, 0xfe);
    streamPut(DEBUG_CHP, 0x1);
}

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
        lcd_clear();
        ltc2990_writereg(LTC2990_TRIGGER, regval, &i2c_errors);
        i2c_report_error(i2c_errors);
        chThdSleepMilliseconds(LTC2990_TRIGGER_WAIT_MS);

        regval = ltc2990_readreg(LTC2990_CONTROL, &i2c_errors);
        i2c_report_error(i2c_errors);

        /* CONTROL check status */
        if(!ltc2990_conversion_done(regval))
        {
            lcd_clear();
            chprintf(DEBUG_CHP, "LTC2990 Error: Conversion not finished");
            chThdSleepS(S2ST(1));
        }
        else
        {
            /* READ ALL */
            ltc2990_read_all(&monitor_data, &i2c_errors);
            i2c_report_error(i2c_errors);

            /* TINT */
            ltc2990_error derror;
            params.tint = ltc2990_calc_tint(&monitor_data, &derror);
            if(derror != LTC2990_OK)
            {
                chprintf(DEBUG_CHP, "TINT ERROR: %d\r\n", derror);
            }

            /* VCC */
            params.vcc = ltc2990_calc_vcc(&monitor_data, &derror );
            if(derror != LTC2990_OK)
            {
                chprintf(DEBUG_CHP, "VCC ERROR: %d\r\n", derror);
            }

            /* Current */
            // chprintf(DEBUG_CHP, "V1_MSB: 0x%x\r\nV1_LSB: 0x%x\r\n", monitor_data.V1_MSB, monitor_data.V1_LSB);
            params.current = solar_v1_calc_current(&monitor_data, &derror);
            if(derror != LTC2990_OK)
            {
                chprintf(DEBUG_CHP, "Current ERROR: %d\r\n", derror);
            }

            /* External Temp */
            // chprintf(DEBUG_CHP, "V3_MSB: 0x%x\r\nV3_LSB: 0x%x\r\n", monitor_data.V3_MSB, monitor_data.V3_LSB);
            params.temp_ext    = solar_v1_calc_temp(&monitor_data, &derror) ;
            if(derror != LTC2990_OK)
            {
                chprintf(DEBUG_CHP, "External T ERROR: %d\r\n", derror);
            }
            lcd_clear();
            chprintf(DEBUG_CHP, "%dC        %dmA  %dmV     %dC", params.temp_ext, params.current, params.vcc, params.tint);
        }
        chThdSleepMilliseconds(1000);
    }
}

static void app_init(void)
{
    // Start up debug output, chprintf(DEBUG_CHP,...)
    sdStart(&DEBUG_SERIAL, &ser_cfg);

    lcd_clear();

    i2cInit();
    i2cStart(&I2CD1, &i2cfg1);
    chprintf(DEBUG_CHP, "I2C1 Initialized.");
    set_util_fwversion(&version_info);
    set_util_hwversion(&version_info);
    chThdSleepS(S2ST(1));

    lcd_clear();
    chprintf(DEBUG_CHP, "FW HASH: %s", version_info.firmware);
    chThdSleepS(S2ST(1));
    lcd_clear();
    // chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x"
             // , version_info.hardware.id_high
             // , version_info.hardware.id_center
             // , version_info.hardware.id_low
            // );
    chThdSleepS(S2ST(1));

    /*
     * Activates CAN driver 1.
     */
    canStart(&CAND1, &cancfg);

}

/*! \brief main application loop
 */
static void main_app(void)
{
    // chprintf(DEBUG_CHP, "app_%s started.\r\n", PROJECT);
    // chprintf(DEBUG_CHP, "\r\n**********\r\n");
    lcd_clear();
    // chprintf(DEBUG_CHP, "OrSat Solar Demo");
    // streamPut(DEBUG_CHP, 0x7c);
    // streamPut(DEBUG_CHP, 0x0a);
    // chThdSleepS(S2ST(10));
    streamPut(DEBUG_CHP, 0x7c);
    streamPut(DEBUG_CHP, 0x82);
    chThdSleepS(S2ST(1));
    chprintf(DEBUG_CHP, "Demo Start");
    // chprintf(DEBUG_CHP, ".");
    // while(1) {

    // chprintf(DEBUG_CHP, "+");
    // chThdSleepS(S2ST(1));

    // }
    chThdSleepS(S2ST(2));
    lcd_clear();
    demo_measure();

    /*
     * Begin main loop
     */
    while (true)
    {
        chThdSleepMilliseconds(1000);
    }
}

int main(void)
{
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();
    app_init();

    main_app();

    return 0;
}

//! @}

