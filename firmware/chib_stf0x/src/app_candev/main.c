/*! \file main.c
 *  app_blinky-just blink a light to test compilers, flash programming and serial terminal tools.
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
 * \defgroup main app_blinky main
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

static SerialConfig ser_cfg =
{
	115200,
	0,
	0,
	0,
};

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

/*
 * Internal loopback mode, 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */


// http://www.bittiming.can-wiki.info/
static const CANConfig cancfg =
{
	// mcr (master control register)
	CAN_MCR_ABOM         //        /*!<Automatic Bus-Off Management */
	| CAN_MCR_AWUM       //        /*!<Automatic Wakeup Mode */
	| CAN_MCR_TXFP,      //        /*!<Transmit FIFO Priority */
	// 0x1230000            // default reset value (No loopback mode)
	// btr  (bit timing register)
	// CAN_BTR_LBKM                   [>!<Loop Back Mode (Debug) <]
	CAN_BTR_SJW(0)  
	| CAN_BTR_TS2(1)  
	| CAN_BTR_TS1(12)  
	| CAN_BTR_BRP(5)  
};

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx_wa, 256);
static THD_FUNCTION(can_rx, p)
{
	CANDriver       *       c = p;
	event_listener_t        el;
	CANRxFrame              rxmsg;
	uint32_t                last = 0;

	(void) p;

	chRegSetThreadName("receiver");
	chEvtRegister(&c->rxfull_event, &el, 0);
	chThdSleepMilliseconds(100);
	while (true)
	{
		if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
		{
			continue;
		}
		chprintf(DEBUG_CHP, "r");
		while (canReceive(c, CAN_ANY_MAILBOX,
		                  &rxmsg, TIME_IMMEDIATE) == MSG_OK)
		{
			/* Process message.*/
			palTogglePad(GPIOB, 3U);
			if(rxmsg.data32[0] != (last + 1))
			{
				chprintf(DEBUG_CHP, "\r\nSEQ ERR %d not %d\r\n", rxmsg.data32[0], last + 1);
			}
			else
			{
				chprintf(DEBUG_CHP, "\r\n\t%d from 0x%x\r\n", rxmsg.data32[0], rxmsg.EID);
			}
			last = rxmsg.data32[0];
		}
	}
	chEvtUnregister(&c->rxfull_event, &el);
}


void CAN_ESR_break(CANDriver *canp) {
    uint32_t esrval = canp->can->ESR;
	chprintf(DEBUG_CHP, "esr:0x%x\r\n", esrval);
}

void CAN_TSR_break(CANDriver *canp) {
	uint32_t tsrval = canp->can->TSR;
	// uint8_t rqcp0 = (tsrval & CAN_TSR_RQCP0);
	// uint8_t txok0 = (tsrval  & CAN_TSR_TXOK0)>>1;
	// uint8_t alst0 = (tsrval  & CAN_TSR_ALST0)>>2;
	// uint8_t terr0 = (tsrval  & CAN_TSR_TERR0)>>3;
	// uint8_t abrq0 = (tsrval  & CAN_TSR_ABRQ0)>>7;
	// uint8_t rqcp1 = (*tsr & CAN_TSR_RQCP1)>>8;
	// uint8_t txok1 = (*tsr & CAN_TSR_TXOK1)>>9;
	// uint8_t alst1 = (*tsr & CAN_TSR_ALST1)>>10;
	// uint8_t terr1 = (*tsr & CAN_TSR_TERR1)>>11;
	// uint8_t abrq1 = (*tsr & CAN_TSR_ABRQ1)>>15;
	// uint8_t rqcp2 = (*tsr & CAN_TSR_RQCP2)>>16;
	// uint8_t txok2 = (*tsr & CAN_TSR_TXOK2)>>17;
	// uint8_t alst2 = (*tsr & CAN_TSR_ALST2)>>18;
	// uint8_t terr2 = (*tsr & CAN_TSR_TERR2)>>19;
	// uint8_t abrq2 = (*tsr & CAN_TSR_ABRQ2)>>23;
	// uint8_t code  = (*tsr & CAN_TSR_ABRQ2)>>24;
	// uint8_t tme0  = (*tsr & CAN_TSR_TME0)>>26;
	// uint8_t tme1  = (*tsr & CAN_TSR_TME1)>>27;
	// uint8_t tme2  = (*tsr & CAN_TSR_TME2)>>28;
	// uint8_t low0  = (*tsr & CAN_TSR_LOW0)>>29;
	// uint8_t low1  = (*tsr & CAN_TSR_LOW1)>>30;
	// uint8_t low2  = (*tsr & CAN_TSR_LOW2)>>31;

	chprintf(DEBUG_CHP, "tsr:0x%x\r\n", tsrval);
	// chprintf(DEBUG_CHP, "rqcp0:0x%x\ttxok0:0x%x\talst0:0x%x\tterr0:0x%x\tabrq0:0x%x\r\n", rqcp0, txok0, alst0,terr0, abrq0 );

}

/*
 * Transmitter thread.
 */
#ifndef MY_CAN_ADDRESS
	#define MY_CAN_ADDRESS 0xCAFE
#endif

static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p)
{
	CANTxFrame txmsg;

	(void)p;
	chRegSetThreadName("transmitter");
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.EID = MY_CAN_ADDRESS;

	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = 8;
	txmsg.data32[0] = 0x1;
	txmsg.data32[1] = 0x00FF00FF;

	chThdSleepMilliseconds(100);
	msg_t msg;
	while (true)
	{
		// chprintf(DEBUG_CHP, "ESR: 0x%x\n\r", *(&CAND1.can->ESR));
		// chprintf(DEBUG_CHP, "TSR: 0x%x\n\r", *(&CAND1.can->TSR));
		CAN_TSR_break(&CAND1);
		chThdSleepMilliseconds(250);
		CAN_ESR_break(&CAND1);
		chThdSleepMilliseconds(750);
		chprintf(DEBUG_CHP, "-");
		msg = canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
		chprintf(DEBUG_CHP, "$");
		chprintf(DEBUG_CHP, "msg: %d\n\r", msg);
		// chprintf(DEBUG_CHP, "ESR: 0x%x\n\r", *((uint32_t)0x40006418));
		// chprintf(DEBUG_CHP, "TSR: 0x%x\n\r", &CAND1.can->TSR);
		txmsg.data32[0] += 0x1;
		chprintf(DEBUG_CHP, "t\t");
	}
}

static void app_init(void)
{
	// start up debug output, chprintf(DEBUG_CHP,...)
	sdStart(&DEBUG_SERIAL, &ser_cfg);

	set_util_fwversion(&version_info);
	set_util_hwversion(&version_info);

	chThdSleepS(S2ST(2));
	chprintf(DEBUG_CHP, "\r\nFirmware Info\r\n");
	chprintf(DEBUG_CHP, "FW HASH: %s\r\n", version_info.firmware);
	chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x\r\n"
	         , version_info.hardware.id_high
	         , version_info.hardware.id_center
	         , version_info.hardware.id_low
	        );

	chprintf(DEBUG_CHP, "\r\nStarting CAN threads...\r\n");

	canStart(&CAND1, &cancfg);

	chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7,
	                  can_rx, (void *)&CAND1);

	chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);

}

/*! \brief main application loop
 */
static void main_app(void)
{

	app_init();
	chprintf(DEBUG_CHP, "App start\r\n");

	while (true)
	{
		chprintf(DEBUG_CHP, "+");
		chThdSleepMilliseconds(500);
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

