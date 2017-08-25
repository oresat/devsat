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

void semtech_test_read(SPIDriver * spip)
{
	uint32_t i=0;
	//Will request the values of some registers, and write the output to serial.
	chprintf(DEBUG_CHP, "\r\nTesting SPI Connection...\r\n");


	//Read one byte using single access mode
	uint8_t txData[2] = {0x01, 0x00};     //Address for RegBitrateMsb 0x02 in read mode (MSB is 0)
	while(1)
	{
		
		
		uint8_t rxData[2] = {0x0, 0x00};     //Received data buffer
		
		//chprintf(DEBUG_CHP, "\r\n%d: Transmitting %x followed by %x\r\n", i++, txData[0], txData[1]);
		spiSelect(spip);

		//We can either use Exchange or Send an Receive
		//spiStartExchange(spip, 2, &txData, &rxData);

		spiStartSend(spip, 1, &txData);

		//Wait for exchange to complete
		while((*spip).state != SPI_READY){}
		

		spiStartReceive(spip, 1, &rxData);

		//Wait for exchange to complete
		while((*spip).state != SPI_READY){}

		spiUnselect(spip);
		//chprintf(DEBUG_CHP, "\r\nValue in register 0x%2.x is 0x%2.x\r\n", txData[0], rxData[0]);
		
        //chThdSleepMilliseconds(750);
		txData[0]++;

		if (txData[0] > 0x70){
			//Pause at the 'beginning' so that I can see the start of the loop on the Salae
			chThdSleepMilliseconds(10000);
			txData[0] = 0x01;
		}
	}


}

void semtech_burst_write(SPIDriver * spip, uint8_t addr, uint8_t * data)
{
	//Will send the array of data over the SPI bus, programming each register
	//Starting from addr, and incrementing.

	uint8_t addrSend = 0b10000000 | addr; //Set MSB to 1 so for write mode


	//TODO: insert addrsend to beginning of data array. OR use spiSend() twice, if that is possible

	spiSelect(spip);

	//Send the address, and then send the data
	spiStartSend(spip, 1, &addrSend);
	//Block until send complete
	while((*spip).state != SPI_READY) {}
	
	spiStartSend(spip, sizeof(data), &data);
	//Block until send complete
	while((*spip).state != SPI_READY) {}

	spiUnselect(spip);

}

void semtech_burst_read(SPIDriver * spip, uint8_t addr, uint8_t * datrcv)
{
	//Reads a series of registers from the semtech

	spiSelect(spip);

	//spiStartExchange(spip, 1, &addr,
	//                 &datrcv); //TODO: Do we use Exchange(), or do we use Write(), immediately followed by Read() without Unselecting?

	spiStartSend(spip, 1, &addr);
	//Wait for send to complete
	while((*spip).state != SPI_READY) { }

	
	spiStartReceive(spip, sizeof(datrcv), datrcv);
	while((*spip).state != SPI_READY) { }
	
	spiUnselect(spip);

}

void semtech_transmit_data(SPIDriver * spip, uint8_t * data)
{
	//Sends data to the FIFO for transmission

	//TODO: Set semtech to tx mode if needed. This clears the FIFO.

	//TODO: Check if FIFO is full (handle excess data if needed)
	//There are 64 bytes in the FIFO (pg. 40)


	//Send data to FIFO
	semtech_burst_write(spip, 0x0, data);

}

void semtech_beacon_transmit(SPIDriver * spip, uint8_t * data)
{
	//The semtech has a beacon mode which periodically transmits a repetitive
	//message. The mode is outlined on page 87 of the datasheet. This function
	//handles the necessary configuration.

}
