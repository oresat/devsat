/*! \file main.c
 *  app_candev
 *
 */

/*
   ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

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
 * \defgroup main app_candev main
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
//#include "util_numbers.h"

//#include "ltc2990.h"
//#include "solar_v1.h"

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

/*
 * Serial configuration
 */
static SerialConfig ser_cfg =
{
    115200,     //Baud rate
    0,          //
    0,          //
    0,          //
};


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

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx_wa, 256);
static THD_FUNCTION(can_rx, p)
{
    event_listener_t        el;
    CANRxFrame              rxmsg;

    (void)p;
    chRegSetThreadName("receiver");

    // Configure Status LED (Green)
    palSetLineMode(LINE_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
    palClearLine(LINE_LED_GREEN);

    // Register RX event
    chEvtRegister(&CAND1.rxfull_event, &el, 0);

    // Start RX Loop
    while(!chThdShouldTerminateX())
    {
        if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
        {
            continue;
        }
        while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK)
        {
            /* Process message.*/
            palToggleLine(LINE_LED_GREEN);
        }
    }

    //Unregister RX event before terminating thread
    chEvtUnregister(&CAND1.rxfull_event, &el);
}

/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p)
{
    CANTxFrame txmsg;

    (void)p;
    chRegSetThreadName("transmitter");

    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x000;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 8;
    txmsg.data8[0] = 0x00;
    txmsg.data8[1] = 0x01;
    txmsg.data8[2] = 0x02;
    txmsg.data8[3] = 0x03;
    txmsg.data8[4] = 0x04;
    txmsg.data8[5] = 0x05;
    txmsg.data8[6] = 0x06;
    txmsg.data8[7] = 0x07;

    // Start TX Loop
    while (!chThdShouldTerminateX())
    {
        //Transmit message
        canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
        chThdSleepMilliseconds(500);
    }
}

static void app_init(void)
{
    // Start up debug output, chprintf(DEBUG_CHP,...)
    sdStart(&DEBUG_SERIAL, &ser_cfg);

    set_util_fwversion(&version_info);
    set_util_hwversion(&version_info);
    chThdSleepS(S2ST(1));

    //Print FW/HW information
    chprintf(DEBUG_CHP, "\r\nFirmware Info\r\n");
    chprintf(DEBUG_CHP, "FW HASH: %s\r\n", version_info.firmware);
    chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x\r\n"
            , version_info.hardware.id_high
            , version_info.hardware.id_center
            , version_info.hardware.id_low
            );

    /*
     * Activates CAN driver 1.
     */
    chprintf(DEBUG_CHP, "\r\nStarting CAN driver...\r\n");
    canStart(&CAND1, &cancfg);


}

static void main_app(void)
{
    /*
     * Starting the transmitter and receiver threads.
     */
    chprintf(DEBUG_CHP, "\r\nStarting RX/TX threads...\r\n");
    chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7, can_rx, NULL);
    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);

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

