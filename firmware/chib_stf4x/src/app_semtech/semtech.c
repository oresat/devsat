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
    uint8_t txData = 0x03;     //Address for RegBitrateMsb 0x02 in read mode (MSB is 0)
    uint8_t rxData = 0x0;     //Received data buffer

    spiSelect(spip);
    //spiStartExchange(spip, 1, &txData, &rxData);
    
    spiStartSend(spip, 1, &txData);
    
    //Wait for exchange to complete
    //while((*spip).state != SPI_READY){} 
        //chprintf(DEBUG_CHP, "\r\n SPI State is: %x", spip->state);
    //}
    
    spiStartReceive(spip, 1, &rxData);
    
    //Wait for exchange to complete
    //while((*spip).state != SPI_READY){ 
    //    chprintf(DEBUG_CHP, "\r\n SPI State is: %x", spip->state);
    //}

    spiUnselect(spip);

    chprintf(DEBUG_CHP, "\r\nValue in RegBitrateMsb is 0x%x\r\n", rxData);
    chprintf(DEBUG_CHP, "\r\nDefault value is 0x1a\r\n");


}

void semtech_burst_write(SPIDriver *spip, uint8_t addr, uint8_t *data){
    //Will send the array of data over the SPI bus, programming each register
    //Starting from addr, and incrementing.

    uint8_t addrSend = 0b10000000 | addr; //Set MSB to 1 so for write mode


    //TODO: insert addrsend to beginning of data array. OR use spiSend() twice, if that is possible
    spiSelect(spip);
    spiStartSend(spip, sizeof(data), &data);

    //Block until send complete
    while((*spip).state != SPI_READY){}

    spiUnselect(spip);

}

uint8_t semtech_single_read(SPIDriver *spip, uint8_t addr){
    //Reads a single register from the semtech
    uint8_t datrcv;
    
    spiSelect(spip);
    spiStartExchange(spip, 1, &addr, &datrcv); //TODO: Do we use Exchange(), or do we use Write(), immediately followed by Read() without Unselecting?
    
    //Wait for exchange to complete
    while((*spip).state != SPI_READY){ }

    spiUnselect(spip);

    return(datrcv);
}

void semtech_transmit_data(SPIDriver *spip, uint8_t *data){
    //Sends data to the FIFO for transmission

    //TODO: Set semtech to tx mode if needed. This clears the FIFO.

    //TODO: Check if FIFO is full (handle excess data if needed)
    //There are 64 bytes in the FIFO (pg. 40)
   

    //Send data to FIFO
    semtech_burst_write(spip, 0x0, data);

}

void semtech_beacon_transmit(SPIDriver *spip, uint8_t *data){
    //The semtech has a beacon mode which periodically transmits a repetitive
    //message. The mode is outlined on page 87 of the datasheet. This function
    //handles the necessary configuration.

}