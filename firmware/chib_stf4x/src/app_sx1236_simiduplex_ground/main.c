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

#define     DEBUG_SERIAL                    SD2
#define     DEBUG_CHP                       ((BaseSequentialStream *) &DEBUG_SERIAL)

#define     F_XOSC                          (32000000U)
#define     F_STEP                          ((double)(61.03515625)) //  (Fxosc/2^19)
#define     APP_CARRIER_FREQ                (436500000U)
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

// static void init_rx_packet(config_sx1236 * s)
// {
// }

// static void init_rx_continuous(config_sx1236 * s)
// {
// }

// static void init_tx_packet(config_sx1236 * s)
// {
// }

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
	radio_mode = 0x01;
	chprintf(DEBUG_CHP, "Radio in TX Mode\r\n");
}



static void init_rx_packet(config_sx1236 * s)
{
    s->Fxosc                            = F_XOSC;
    s->Fstep                            = F_STEP;
    s->carrier_freq                     = APP_CARRIER_FREQ;
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
	//sx1236_print_regs(&SPID1);
	radio_mode = 0x00;
	chprintf(DEBUG_CHP, "Radio in RX Mode\r\n");
}

/*
 * GPT3 callback.

static void gpt3cb(GPTDriver *gptp) {
	
	(void)gptp;
	uint8_t value=0;
 	value = sx1236_read_FIFO(&SPID1);
	chprintf(DEBUG_CHP, " %x \r\n", value);

}


 * GPT3 configuration.
 
static const GPTConfig gpt3cfg = {
	1000,    // 4.8kHz timer clock.
	gpt3cb,  // Timer callback.
	0,
	0
};
 */
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


}



static THD_WORKING_AREA(waThread_sx1236_rx, 512);
static THD_FUNCTION(Thread_sx1236_rx, arg)
{
    (void) arg;
	uint8_t value=0;
    event_listener_t evl_dio0, evl_dio1, evl_dio2, evl_dio3, evl_dio4, evl_dio5;

    chRegSetThreadName("sx1236_rx");

    chEvtRegister(&dio0_event,           &evl_dio0,         0);
    chEvtRegister(&dio1_event,           &evl_dio1,         1);
    chEvtRegister(&dio2_event,           &evl_dio2,         2);
    chEvtRegister(&dio3_event,           &evl_dio3,         3);
    chEvtRegister(&dio4_event,           &evl_dio4,         4);
    chEvtRegister(&dio5_event,           &evl_dio5,         5);

    chprintf(DEBUG_CHP, "Thread started: %s\r\n", "sx1236_dio");
    while (TRUE)
    {
        //chEvtDispatch(evhndl_sx1236_dio, chEvtWaitOneTimeout(EVENT_MASK(3), MS2ST(50)));
		
    	/* Waiting for any of the events we're registered on.*/
    	eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

    	/* Serving events.*/
    	if (evt & EVENT_MASK(3)) {
			if (radio_mode==0x00){
				while ( !palReadPad(GPIOC, GPIOC_SX_DIO3)){			//fifo not empty
	  				value = sx1236_read_FIFO(&SPID1);
					chprintf(DEBUG_CHP, "## 0x%x ##\r\n", value);
 				}
			}	
    	}

    }
}


static void main_loop(void)
{
    chThdSleepMilliseconds(500);
    //chprintf(DEBUG_CHP, "\r\n");
    //chprintf(DEBUG_CHP, "**INFO** SX1236 RX Test...\r\n");
    //chprintf(DEBUG_CHP, "\r\n");
    //sx1236_check_reg(&SPID1, regaddrs.RegVersion, 0x12);

    init_rx_packet(&dut_config);
    chThdSleepMilliseconds(500);
	uint8_t value=0;
	uint8_t inst=0x01;
    // chprintf(DEBUG_CHP, "**INFO**\r\n");
    // sx1236_print_regs(&SPID1);

	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT );
	
	//empty fifo
	while ( !palReadPad(GPIOC, GPIOC_SX_DIO3)){			//fifo not empty
		value = sx1236_read_FIFO(&SPID1);
		chprintf(DEBUG_CHP, "empty fifo: %x \r\n", value);
 	}	

    while (true)
    {
        chThdSleepMilliseconds(2000);
        chprintf(DEBUG_CHP, ".");
		
		if (!palReadPad(GPIOC, GPIOC_BUTTON)){
			init_tx_packet(&dut_config);
			//sx1236_reset() ;
			chprintf(DEBUG_CHP, "* \n");
			//palTogglePad(GPIOA, GPIOA_SX_TESTOUT);
			chThdSleepMilliseconds(50);
			sx1236_create_instruction_packet_tx(&SPID1, inst);
			chThdSleepMilliseconds(200);
			//clean up fifo
			sx1236_reset() ;
			while ( !palReadPad(GPIOC, GPIOC_SX_DIO3)){			//fifo not empty
				value = sx1236_read_FIFO(&SPID1);
				//chprintf(DEBUG_CHP, "fifo cleanup: %x \r\n", value);
 			}	
			init_rx_packet(&dut_config);
			//sx1236_reset() ;
		}		
    }
}

int main(void)
{
    halInit();
    chSysInit();
    app_init();

    // Enable interrupt through the EXT interface
    extStart(&EXTD1, &extcfg);

    chThdCreateStatic(waThread_sx1236_rx,      sizeof(waThread_sx1236_rx),   NORMALPRIO, Thread_sx1236_rx, NULL);
    main_loop();
    return 0;
}



