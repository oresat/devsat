/* 
    Code for communication with Semtech SX1236 using SPI and GPIO

    Written by Evan Yand et.al. for the Portland State Aerospace Society

    Licensed GPL v3 August 2017
*/

#include "ch.h"
#include "chprintf.h"
#include "board.h"
#include "hal.h"


#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

void semtech_test_read(SPIDriver *spip){
    //Will request the values of some registers, and write the output to serial.
    chprintf(DEBUG_CHP, "\r\nTesting SPI Connection...\r\n");

    //Read one byte using single access mode
    int txData = 0b00000010;     //Address for RegBitrateMsb 0x02 in read mode (MSB is 0)
    int rxData = 0b00000000;     //Received data buffer

    spiSelect(&SPID1);
    spiStartExchange(&SPID1, 1, &txData, &rxData);
    
    //Wait for exchange to complete
    while(SPID1.state != SPI_READY){ }


    spiUnselect(&SPID1);

    chprintf(DEBUG_CHP, "\r\nValue in RegBitrateMsb is 0x%x\r\n", &rxData);
    chprintf(DEBUG_CHP, "\r\nDefault value is 0x1a\r\n");


}
