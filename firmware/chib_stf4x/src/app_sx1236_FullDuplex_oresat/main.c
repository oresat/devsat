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
#define     APP_CARRIER_FREQ_GND            (435500000U)
#define     APP_CARRIER_FREQ_SAT            (437500000U)
#define     APP_FREQ_DEV                    (5000U)

// #define     APP_BITRATE                     (4800)
#define     APP_BITRATE                     (1200U)

// RegPaConfig
#define     PA_MAXPOWER                     ((uint8_t)(0x0))
#define     PA_OUTPOWER                     ((uint8_t)(0x0))

// RSSI Thresh
#define     RSSI_THRESH                     ((uint8_t)(0x70U))

// SeqConfig1
#define     FromTransmit_RX                 ((uint8_t)(0b1<<0))
#define     FromIdle_RX                     ((uint8_t)(0b1<<1))
#define     LowPowerSelect_IDLE             ((uint8_t)(0b1<<2))
#define     FromStart_TO_LP                 ((uint8_t)(0b00<<3))
#define     FromStart_TO_RX                 ((uint8_t)(0b01<<3))
#define     FromStart_TO_TX                 ((uint8_t)(0b10<<3))
#define     FromStart_TO_TX_FIFOINT         ((uint8_t)(0b11<<3))
#define     Idle_TO_STANDBY                 ((uint8_t)(0b00<<5))
#define     SEQ_STOP                        ((uint8_t)(0b1<<6))
#define     SEQ_START                       ((uint8_t)(0b1<<7))

// SeqConfig1
#define     FromRX_PKT_RX_PLD_RDY           ((uint8_t)(0b001<<5))
#define     FromRX_PKT_RX_CRC_OK            ((uint8_t)(0b011<<5))
#define     FromRX_Timeout_TO_RX_ST         ((uint8_t)(0b00<<3))
#define     FromPKT_RXD_TO_RX               ((uint8_t)(0b100<<0))
#define     FromPKT_RXD_TO_LP_SELECT        ((uint8_t)(0b010<<0))

// Sync bytes
#define     SX1236_SYNCVALUE1               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE2               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE3               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE4               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE5               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE6               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE7               ((uint8_t) (0xe7U))
#define     SX1236_SYNCVALUE8               ((uint8_t) (0xe7U))

// Payload length
#define     PAYLOAD_LENGTH                  ((uint8_t) (0x05U))
#define     FIFO_THRESH                     ((uint8_t) (0x05U))

// Structure to hold configuration for test
static config_sx1236 dut_config ;
uint8_t radio_mode;

static SerialConfig ser_cfg =
{
    115200,     //Baud rate
    0,          //
    0,          //
    0,          //
};

/*
 * Receive on SPI bus: sx1236
 */
static const SPIConfig spicfg_rx =
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
 * Transmit on SPI bus: sx1236
 */
static const SPIConfig spicfg_tx =
{
    NULL,               // Operation complete callback
    GPIOB,              // Slave select port
    GPIOB_SPI2_NSS,     // Slave select pad
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


static void init_rx_packet(config_sx1236 * s)
{
    s->Fxosc                            = F_XOSC;
    s->Fstep                            = F_STEP;
    s->carrier_freq                     = APP_CARRIER_FREQ_GND;
    s->freq_dev_hz                      = APP_FREQ_DEV;
    s->bitrate                          = APP_BITRATE;

    sx1236_init_state(&s->sx1236_state);

    s->sx1236_state.RegOpMode          = 0x0 | SX1236_LOW_FREQ_MODE | SX1236_FSK_MODE |  SX1236_RECEIVER_MODE;
    //s->sx1236_state.RegOsc             = 0x0 | SX1236_OSC_DIV_8 ;
    s->sx1236_state.RegPacketConfig1   	= 0x00 | SX1236_FIXED_PACKET | SX1236_CRC_ON ;
	s->sx1236_state.RegPacketConfig2   	= 0x00 | SX1236_PACKET_MODE ;
	s->sx1236_state.RegPayloadLength   	= 0x20;
	//s->sx1236_state.RegOokPeak   		= 0x08;				//disable syncronizer bit

	sx1236_configure(&SPID1, s);
	chprintf(DEBUG_CHP, "Radio in RX Mode\r\n");
	sx1236_print_regs(&SPID1);

}


static void init_tx_packet(config_sx1236 * s)
{
    s->Fxosc                            = F_XOSC;
    s->Fstep                            = F_STEP;
    s->carrier_freq                     = APP_CARRIER_FREQ_SAT;
    s->freq_dev_hz                      = APP_FREQ_DEV;
    s->bitrate                          = APP_BITRATE;

    sx1236_init_state(&s->sx1236_state);

    s->sx1236_state.RegOpMode          	= 0x00 | SX1236_LOW_FREQ_MODE | SX1236_FSK_MODE |  SX1236_TRANSMITTER_MODE ;
    //s->sx1236_state.RegOsc            = 0x00 | SX1236_OSC_DIV_8 ;		//FXOSC is diabled by default
    s->sx1236_state.RegPacketConfig1   	= 0x00 | SX1236_FIXED_PACKET | SX1236_CRC_ON ;
	s->sx1236_state.RegPacketConfig2   	= 0x00 | SX1236_PACKET_MODE ;
	s->sx1236_state.RegPayloadLength   	= 0x20;
	//s->sx1236_state.RegOokPeak   		= 0x08;				//disable syncronizer bit

	sx1236_configure(&SPID2, s);
	chprintf(DEBUG_CHP, "Radio in TX Mode\r\n");
	sx1236_print_regs(&SPID2);
}



/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx_wa, 1024);
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
				sx1236_create_data_packet_tx(&SPID2, packet_data, sizeof(packet_data));
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

    chprintf(DEBUG_CHP, "TSR:0x%x\r\n", tsrval);
    // chprintf(DEBUG_CHP, "rqcp0:0x%x\ttxok0:0x%x\talst0:0x%x\tterr0:0x%x\tabrq0:0x%x\r\n", rqcp0, txok0, alst0,terr0, abrq0 );

}

/*
 * Transmitter function.
 */
// change msg_t to malay_msg_t
msg_t CAN_tx(sx1236_packet p)
{
    CANTxFrame txmsg;
	msg_t msg;

    chRegSetThreadName("transmitter");
    txmsg.IDE = CAN_IDE_EXT;
    txmsg.EID = 0x11;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 8;
    txmsg.data8[0] = p.PacInstruction;
    txmsg.data8[1] = p.PacData[0];
    txmsg.data8[2] = p.PacData[1];
    txmsg.data8[3] = p.PacData[2];
    txmsg.data8[4] = p.PacData[3];
    txmsg.data8[5] = p.PacData[4];
    txmsg.data8[6] = p.PacData[5];
    txmsg.data8[7] = p.PacData[6];

	//chprintf(DEBUG_CHP, "sent CAN packet\r\n");
    msg = canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
	// check this message for semantics during debugging
	return msg;
    
}


static THD_WORKING_AREA(waThread_sx1236_rx, 2048);
static THD_FUNCTION(Thread_sx1236_rx, arg)
{
    (void) arg;
	uint8_t value;
    sx1236_raw_packet raw_packet;
    sx1236_packet formatted_packet;
    event_listener_t evl_dio0, evl_dio1, evl_dio2, evl_dio3, evl_dio4, evl_dio5;

    chRegSetThreadName("sx1236_dio");

    //chEvtRegister(&dio0_event_sx2,           &evl_dio0,         5);
    //chEvtRegister(&dio1_event_sx2,           &evl_dio1,         6);
    //chEvtRegister(&dio2_event_sx2,           &evl_dio2,         7);
    chEvtRegister(&dio3_event_sx2,           &evl_dio3,         0);
    //chEvtRegister(&dio4_event_sx2,           &evl_dio4,         9);
    //chEvtRegister(&dio5_event_sx2,           &evl_dio5,         10);

    chprintf(DEBUG_CHP, "Thread started: %s\r\n", "sx1236_dio");

	/*
	while (TRUE){
	value = sx1236_read_FIFO(&SPID1);
	chprintf(DEBUG_CHP, "\r\r## 0x%x ##\r\n", value);
	chprintf(DEBUG_CHP, "not DIO3: %x \r\n", !palReadPad(GPIOC, GPIOC_SX_DIO3));
	chprintf(DEBUG_CHP, "DIO3: %x \r\n", palReadPad(GPIOC, GPIOC_SX_DIO3));

	chprintf(DEBUG_CHP, "not 2 DIO3: %x \r\n", !palReadPad(GPIOC, GPIOC_SX2_DIO3));
	chprintf(DEBUG_CHP, "2 DIO3: %x \r\n", palReadPad(GPIOC, GPIOC_SX2_DIO3));
	chThdSleepMilliseconds(500);
	}*/


    while (TRUE)
    {
        //chEvtDispatch(evhndl_sx1236_dio, chEvtWaitOneTimeout(EVENT_MASK(3), MS2ST(50)));
        /* Waiting for any of the events we're registered on.*/
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
		//chprintf(DEBUG_CHP, "Interrupt happened %x and mask %x\r\n", evt,EVENT_MASK(3));

        /* Serving events.*/
        if (evt) {
			 //chprintf(DEBUG_CHP, "I am in loop \r\n");
             while ( !palReadPad(GPIOC, GPIOC_SX2_DIO3)) {			//fifo not empty
                    sx1236_packet_rx2(&SPID1, &dut_config, &raw_packet);
                    sx1236_packet_format(&formatted_packet, &raw_packet);
                    chprintf(DEBUG_CHP, "packet received\r\n");
                    CAN_tx(formatted_packet);
            }
        }

    }
}



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

    spiStart(&SPID1, &spicfg_rx);
	chThdSleepMilliseconds(500);
	spiStart(&SPID2, &spicfg_tx);

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

    // Enable interrupt through the EXT interface
    extStart(&EXTD1, &extcfg);


    //chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);
    chThdSleepMilliseconds(500);
    chprintf(DEBUG_CHP, "\r\n");
    //sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x12);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT );

	palSetPadMode(GPIOC, 10, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 11, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 12, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 13, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 14, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 15, PAL_MODE_INPUT );

		
    chThdSleepMilliseconds(500);

    init_rx_packet(&dut_config);
	chThdSleepMilliseconds(500);	
    init_tx_packet(&dut_config);
	chThdSleepMilliseconds(500);
	uint8_t value=0;
    // chprintf(DEBUG_CHP, "**INFO**\r\n");
    // sx1236_print_regs(&SPID1);

	chprintf(DEBUG_CHP, "not DIO3: %x \r\n", !palReadPad(GPIOC, GPIOC_SX_DIO3));
	chprintf(DEBUG_CHP, "DIO3: %x \r\n", palReadPad(GPIOC, GPIOC_SX_DIO3));

	chprintf(DEBUG_CHP, "not 2 DIO3: %x \r\n", !palReadPad(GPIOC, GPIOC_SX2_DIO3));
	chprintf(DEBUG_CHP, "2 DIO3: %x \r\n", palReadPad(GPIOC, GPIOC_SX2_DIO3));

	//empty fifo
	while ( !palReadPad(GPIOC, GPIOC_SX_DIO3)){			//fifo not empty
		value = sx1236_read_FIFO(&SPID2);
		chprintf(DEBUG_CHP, "empty fifo: %x \r\n", value);
 	}	

	while ( !palReadPad(GPIOC, GPIOC_SX2_DIO3)){			//fifo not empty
		value = sx1236_read_FIFO(&SPID1);
		chprintf(DEBUG_CHP, "empty fif2: %x \r\n", value);
 	}		
    
	chThdCreateStatic(waThread_sx1236_rx,      sizeof(waThread_sx1236_rx),   NORMALPRIO, Thread_sx1236_rx, NULL);
    chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7, can_rx, NULL);
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



