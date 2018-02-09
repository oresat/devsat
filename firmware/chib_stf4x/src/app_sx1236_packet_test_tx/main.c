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

// RegPaConfig
#define     PA_MAXPOWER                     ((uint8_t)(0x0))
#define     PA_OUTPOWER                     ((uint8_t)(0x0))

// RSSI Thresh
#define     RSSI_THRESH                     ((uint8_t)(0x70U))



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



static void init_tx_packet(config_sx1236 * s)
{
    s->Fxosc                            = F_XOSC;
    s->Fstep                            = F_STEP;
    //s->carrier_freq                     = APP_CARRIER_FREQ;
    //s->freq_dev_hz                      = APP_FREQ_DEV;
    //s->bitrate                          = APP_BITRATE;

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
 * Receiver thread.
 */
static THD_WORKING_AREA(waThread_sx1236_tx, 512);
static THD_FUNCTION(Thread_sx1236_tx, p)
{
    (void) p;
    chThdSleepMilliseconds(500);
    init_tx_packet(&dut_config);
    chThdSleepMilliseconds(500);
    uint8_t value=0;
    uint8_t packet_data[26];
    int i, j;

    for(i=0;i==25;i++)
    {
	value=0;
        for(j=0;j==2000;j++)
        {
            /* Process message.*/
            if (j<1000)
            {				
		//create packet data for transmission
		packet_data[0] = value;
		value++;
            }
	    else
	    {
		packet_data[0] = 0xff;

	    }
            sx1236_create_data_packet_tx(&SPID1, packet_data, sizeof(packet_data));
        }
        dut_config.carrier_freq                     = APP_CARRIER_FREQ;
        dut_config.freq_dev_hz                      = APP_FREQ_DEV;
        dut_config.bitrate                          = APP_BITRATE;
        init_tx_packet(&dut_config);
        
    }

}




static THD_WORKING_AREA(waThread_sx1236_rx, 512);
static THD_FUNCTION(Thread_sx1236_rx, arg)
{
    (void) arg;
    sx1236_raw_packet raw_packet;
    sx1236_packet formatted_packet;
    event_listener_t evl_dio0, evl_dio1, evl_dio2, evl_dio3, evl_dio4, evl_dio5;

    chRegSetThreadName("sx1236_dio");

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
            if (radio_mode==0x00) {
                while ( !palReadPad(GPIOC, GPIOC_SX_DIO3)) {			//fifo not empty
                    sx1236_packet_rx(&SPID1, &dut_config, &raw_packet);
                    sx1236_packet_format(&formatted_packet, &raw_packet);
                    chprintf(DEBUG_CHP, "packet received\r\n");
                }
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

    spiStart(&SPID1, &spicfg);

    dio_init();

    chprintf(DEBUG_CHP, "Reset sx1236\r\n");
    sx1236_reset() ;

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

    init_rx_packet(&dut_config);
	chThdSleepMilliseconds(500);
	uint8_t value=0;
    // chprintf(DEBUG_CHP, "**INFO**\r\n");
    // sx1236_print_regs(&SPID1);
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT );
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT );
	
	//empty fifo
	while ( !palReadPad(GPIOC, GPIOC_SX_DIO3)){			//fifo not empty
		value = sx1236_read_FIFO(&SPID1);
		chprintf(DEBUG_CHP, "empty fifo: %x \r\n", value);
 	}	
    
	chThdCreateStatic(waThread_sx1236_rx,      sizeof(waThread_sx1236_rx),   NORMALPRIO, Thread_sx1236_rx, NULL);
    chThdCreateStatic(waThread_sx1236_tx,      sizeof(waThread_sx1236_tx),   NORMALPRIO, Thread_sx1236_tx, NULL);
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



