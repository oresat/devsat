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
#include "board.h"

#include "util_general.h"
#include "util_version.h"
#include "util_numbers.h"

//#include "ltc2990.h"
//#include "solar_v1.h"
#include "semtech.c"
#include "sx1236.h"

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)
//#define 	STM32_SPI_USE_SPI1   FALSE
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
    CAN_BTR_TS1(14)   |     //Time Segment 1
    CAN_BTR_TS2(1)    |     //Time Segment 2
    CAN_BTR_BRP(4)          //Bit Rate Prescaler
};

/*
 * SPI Configuration
 */
static const SPIConfig spicfg = {
    NULL, // Operation complete callback
    GPIOA,     // Slave select port
    GPIOA_SPI1_NSS,     // Slave select pad
    // SPI cr1 data (see 446 ref man.)
    SPI_CR1_SPE     |// SPI enable
    SPI_CR1_MSTR    |// Master
    //SPI_CR1_BR_2    |
    SPI_CR1_BR_1    |
    SPI_CR1_BR_0    |
    SPI_CR1_SSM,    // SPI baudrate
    0//SPI_CR2_SSOE     // Set SSOE in cr2

};



static void app_init(void)
{
    // Start up debug output, chprintf(DEBUG_CHP,...)
    sdStart(&DEBUG_SERIAL, &ser_cfg);

    set_util_fwversion(&version_info);
    set_util_hwversion(&version_info);
    chThdSleepS(S2ST(2));

    //Print FW/HW information
    chprintf(DEBUG_CHP, "\r\nFirmware Info\r\n");
    chprintf(DEBUG_CHP, "FW HASH: %s\r\n", version_info.firmware);
    chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x\r\n"
            , version_info.hardware.id_high
            , version_info.hardware.id_center
            , version_info.hardware.id_low
            );

    /*
     * Initialize all drivers
     */
    // CAN Driver 1
    canStart(&CAND1, &cancfg);
    // SPI Driver 1
    spiStart(&SPID1, &spicfg);

}

int main(void) {
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

/* Disabled for now
    *
     * Starting the working threads.
    /
    chprintf(DEBUG_CHP, "\r\nStarting threads...\r\n");
    chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7, can_rx, NULL);
    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);
*/
    //semtech_test_read(&SPID1);
    
    uint8_t address = 0x3;

    semtech_burst_write(&SPID1, address,  0x1a, 1);

    semtech_test_read(&SPID1, 0x3);
    /*
     * Begin main loop
     */
    while (true)
    {
        chThdSleepMilliseconds(500);
        //Test SPI connectivity
    }

    return 0;
}
