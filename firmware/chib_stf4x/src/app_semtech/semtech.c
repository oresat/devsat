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



}
