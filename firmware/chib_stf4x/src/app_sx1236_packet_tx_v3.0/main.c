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

#include "ltc2990.h"
#include "solar_v1.h"

#define     DEBUG_SERIAL                    SD2
#define     DEBUG_CHP                       ((BaseSequentialStream *) &DEBUG_SERIAL)

#define     F_XOSC                          (32000000U)
#define     F_STEP                          ((double)(61.03515625)) //  (Fxosc/2^19)
#define     APP_CARRIER_FREQ                (436500000U)
#define     APP_FREQ_DEV                    (5000U)

// #define     APP_BITRATE                     (4800)
#define     APP_BITRATE                     (1200U)

// Payload length
#define     PAYLOAD_LENGTH                  ((uint8_t) (0x05U))
#define     FIFO_THRESH                     ((uint8_t) (0x05U))

// Structure to hold configuration for test
static config_sx1236 dut_config ;


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
    CAN_BTR_TS1(14)   |     //Time Segment 1
    CAN_BTR_TS2(1)    |     //Time Segment 2
    CAN_BTR_BRP(4)          //Bit Rate Prescaler
};


static void init_tx_packet(config_sx1236 * s)
{
    s->Fxosc                            = F_XOSC;
    s->Fstep                            = F_STEP;
    s->carrier_freq                     = APP_CARRIER_FREQ;
    s->freq_dev_hz                      = APP_FREQ_DEV;
    s->bitrate                          = APP_BITRATE;

    sx1236_init_state(&s->sx1236_state);

    s->sx1236_state.RegOpMode          	= 0x00 | SX1236_LOW_FREQ_MODE | SX1236_FSK_MODE |  SX1236_TRANSMITTER_MODE ;
    //s->sx1236_state.RegOsc            = 0x00 | SX1236_OSC_DIV_8 ;		//FXOSC is diabled by default
    s->sx1236_state.RegPacketConfig1   	= 0x00 | SX1236_FIXED_PACKET | SX1236_CRC_ON ;
	s->sx1236_state.RegPacketConfig2   	= 0x00 | SX1236_PACKET_MODE ;
	s->sx1236_state.RegPayloadLength   	= 0x20;
	//s->sx1236_state.RegOokPeak   		= 0x08;				//disable syncronizer bit

	sx1236_configure(&SPID1, s);
	//sx1236_print_regs(&SPID1);
}





/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx_wa, 512);
static THD_FUNCTION(can_rx, p)
{
    event_listener_t        el;
    CANRxFrame              rxmsg;
    ltc2990_data            telemetry;
    ltc2990_error           derror;
    solar_v1_p              params;
	
	uint8_t 				packet_data[26];	

    (void)p;
    chRegSetThreadName("receiver");

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
            if (0x30 & rxmsg.EID)
            {
				
                telemetry.T_INT_MSB = rxmsg.data8[0];
                telemetry.T_INT_LSB = rxmsg.data8[1];
                telemetry.VCC_MSB = rxmsg.data8[2];
                telemetry.VCC_LSB = rxmsg.data8[3];
                telemetry.V1_MSB = rxmsg.data8[4];
                telemetry.V1_LSB = rxmsg.data8[5];
                telemetry.V3_MSB = rxmsg.data8[6];
                telemetry.V3_LSB = rxmsg.data8[7];
                params.tint = ltc2990_calc_tint(&telemetry, &derror);
                params.vcc = ltc2990_calc_vcc(&telemetry, &derror);
                params.current = solar_v1_calc_current(&telemetry, &derror);
                params.temp_ext = solar_v1_calc_temp(&telemetry, &derror);
                chprintf(DEBUG_CHP, "\r\n0x%x:\r\nExt Temp: %dC\r\nCurrent: %dmA\r\nVoltage: %dmV\r\nInt Temp: %dC\r\n", rxmsg.EID, params.temp_ext, params.current, params.vcc, params.tint);
				

				//create packet data for transmission
				packet_data[0] = rxmsg.data8[0];
				packet_data[1] = rxmsg.data8[1];
                packet_data[2] = rxmsg.data8[2];
                packet_data[3] = rxmsg.data8[3];
                packet_data[4] = rxmsg.data8[4];
                packet_data[5] = rxmsg.data8[5];
                packet_data[6] = rxmsg.data8[6];
                packet_data[7] = rxmsg.data8[7];
 				//chprintf(DEBUG_CHP, "\r\n***0x%x,  0x%x,   0x%x,  0x%x,  0x%x,  0x%x,  0x%x,  0x%x***\r\n", rxmsg.data8[0], rxmsg.data8[1], rxmsg.data8[2], rxmsg.data8[3], rxmsg.data8[4], rxmsg.data8[5], rxmsg.data8[6], rxmsg.data8[7]);
				sx1236_create_data_packet_tx(&SPID1, packet_data, sizeof(packet_data));
            }
        }
    }

    //Unregister RX event before terminating thread
    chEvtUnregister(&CAND1.rxfull_event, &el);
}


//Process Error Status Register
void CAN_ESR_break(CANDriver *canp) {
    uint32_t esrval = canp->can->ESR;
    chprintf(DEBUG_CHP, "ESR:0x%x\r\n", esrval);
}

//Process Transmit Status Register
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

    chprintf(DEBUG_CHP, "TSR:0x%x\r\n", tsrval);
    // chprintf(DEBUG_CHP, "rqcp0:0x%x\ttxok0:0x%x\talst0:0x%x\tterr0:0x%x\tabrq0:0x%x\r\n", rqcp0, txok0, alst0,terr0, abrq0 );

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
    txmsg.IDE = CAN_IDE_EXT;
    txmsg.EID = 0x11;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 8;
    txmsg.data8[0] = 0x00;
    txmsg.data8[1] = 0x00;
    txmsg.data8[2] = 0x00;
    txmsg.data8[3] = 0x00;
    txmsg.data8[4] = 0x00;
    txmsg.data8[5] = 0x00;
    txmsg.data8[6] = 0x00;
    txmsg.data8[7] = 0x00;

    // Start TX Loop
    while (!chThdShouldTerminateX())
    {
        //Process TSR and ESR
        /*chprintf(DEBUG_CHP, "\n\rStatus:\n\r");*/
        /*CAN_TSR_break(&CAND1);*/
        chThdSleepMilliseconds(250);
        /*CAN_ESR_break(&CAND1);*/
        chThdSleepMilliseconds(750);

        //Transmit message
        canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
        /*chprintf(DEBUG_CHP, "TX msg: %d\n\r", msg);*/
    }
}

/*
static THD_WORKING_AREA(waThread_sx1236_tx, 512);
static THD_FUNCTION(Thread_sx1236_tx, arg)
{
	(void) arg;

	//palSetPadMode(GPIOC, 1, PAL_MODE_OUTPUT_PUSHPULL );
	//palSetPadMode(GPIOC, 2, PAL_MODE_OUTPUT_PUSHPULL );
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT );
	uint16_t packet_length = 0x10;
	uint8_t value = 0;
	uint16_t count = 0;
	chThdSleepMilliseconds(2000);

	
	gptStart(&GPTD3, &gpt3cfg);
	chThdSleepMilliseconds(200);
	gptStartContinuous(&GPTD3, 1000);
	

	while (true)
    {
		if ( palReadPad(GPIOC, GPIOC_SX_DIO2)){
	  		chprintf(DEBUG_CHP, "FIFO Full\r\n");
			chThdSleepMilliseconds(10);
		}
		if ( !palReadPad(GPIOC, GPIOC_SX_DIO2)){
		  if(count < packet_length){
			value=value+1;
			count=count+1;
			sx1236_write_FIFO(&SPID1,value);
			chprintf(DEBUG_CHP, "s");
		  }else{
		  count=0;
		  chThdSleepMilliseconds(1500);
		  }
		}
		
		
    }

}
*/

static void app_init(void)
{
    // Start up debug output, chprintf(DEBUG_CHP,...)
    sdStart(&DEBUG_SERIAL, &ser_cfg);

    set_util_fwversion(&version_info);
    set_util_hwversion(&version_info);


    //Print FW/HW information
    chprintf(DEBUG_CHP, "\r\nFirmware Info\r\n");
    chprintf(DEBUG_CHP, "FW HASH: %s\r\n", version_info.firmware);
    chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x\r\n"
             , version_info.hardware.id_high
             , version_info.hardware.id_center
             , version_info.hardware.id_low
            );

    spiStart(&SPID1, &spicfg);

    dio_init();

    chprintf(DEBUG_CHP, "Reset sx1236\r\n");
    sx1236_reset() ;

    // CAN Driver 1
    canStart(&CAND1, &cancfg);

}


int main(void)
{
    halInit();
    chSysInit();
    app_init();

	//chThdCreateStatic(waThread_sx1236_tx,      sizeof(waThread_sx1236_tx),   NORMALPRIO, Thread_sx1236_tx, NULL);
    chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7, can_rx, NULL);
    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);
    chThdSleepMilliseconds(500);
    chprintf(DEBUG_CHP, "\r\n");
    //sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x12);

    init_tx_packet(&dut_config);
    

    // chprintf(DEBUG_CHP, "**INFO**\r\n");
    // sx1236_print_regs(&SPID1);

    while (true)
    {
        chThdSleepMilliseconds(500);
        palTogglePad(GPIOA, GPIOA_SX_TESTOUT);
        chprintf(DEBUG_CHP, ".");
    }
    return 0;
}



