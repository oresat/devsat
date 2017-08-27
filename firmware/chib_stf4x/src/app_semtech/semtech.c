/*
    Code for communication with Semtech SX1236 using SPI and GPIO

    Written by Evan Yand et.al. for the Portland State Aerospace Society

    Licensed GPL v3 August 2017
*/

#include "ch.h"
#include "chprintf.h"
#include "board.h"
#include "hal.h"
#include "sx1236.h"


#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)


/* assign the addresses to struct for registers in transceiver block */
struct SX1236 transceiver = {
	.RegFifo 			= 0x00,
	.RegOpMode 			= 0x01,	
	.RegBitrateMsb 		= 0x02,
	.RegBitrateLsb 		= 0x03,
	.RegFdevMsb 		= 0x04,
	.RegFdevLsb 		= 0x05,
	.RegFrfMsb 			= 0x06,
	.RegFrfMid 			= 0x07,
	.RegFrfLsb 			= 0x08,
	.RegPaConfig 		= 0x09,
	.RegPaRamp 			= 0x0A,
	.RegOcp 			= 0x0B,
	.RegLna 			= 0x0C,
	.RegRxConfig 		= 0x0D,
	.RegRssiConfig 		= 0x0E,
	.RegRssiCollision 	= 0x0F,
	.RegRssiThresh 		= 0x10,
	.RegRssiValue 		= 0x11,
	.RegRxBw 			= 0x12,
	.RegAfcBw 			= 0x13,
	.RegOokPeak 		= 0x14,
	.RegOokFix 			= 0x15,
	.RegOokAvg 			= 0x16,
	.RegAfcFei 			= 0x1A,
	.RegAfcMsb 			= 0x1B,
	.RegAfcLsb 			= 0x1C,
	.RegFeiMsb 			= 0x1D,
	.RegFeiLsb 			= 0x1E,
	.RegPreambleDetect 	= 0x1F,
	.RegRxTimeout1 		= 0x20,
	.RegRxTimeout2 		= 0x21,
	.RegRxTimeout3 		= 0x22,
	.RegRxDelay 		= 0x23,
	.RegOsc 			= 0x24,
	.RegPreambleMsb 	= 0x25,
	.RegPreambleLsb 	= 0x26,
	.RegSyncConfig 		= 0x27,
	.RegSyncValue1 		= 0x28,
	.RegSyncValue2 		= 0x29,
	.RegSyncValue3 		= 0x2A,
	.RegSyncValue4 		= 0x2B,
	.RegSyncValue5 		= 0x2C,
	.RegSyncValue6 		= 0x2D,
	.RegSyncValue7 		= 0x2E,
	.RegSyncValue8 		= 0x2F,
	.RegPacketConfig1 	= 0x30,
	.RegPacketConfig2 	= 0x31,
	.RegPayloadLength 	= 0x32,
	.RegNodeAdrs 		= 0x33,
	.RegBroadcastAdrs 	= 0x34,
	.RegFifoThresh 		= 0x35,
	.RegSeqConfig1 		= 0x36,
	.RegSeqConfig2 		= 0x37,
	.RegTimerResol 		= 0x38,
	.RegTimer1Coef		= 0x39,
	.RegTimer2Coef		= 0x3A,
	.RegImageCal 		= 0x3B,
	.RegTemp 			= 0x3C,
	.RegLowBat 			= 0x3D,
	.RegIrqFlags1 		= 0x3E,
	.RegIrqFlags2 		= 0x3F,
	.RegDioMapping1 	= 0x40,
	.RegDioMapping2		= 0x41,
	.RegVersion 		= 0x42,
	.RegPllHop 			= 0x44,
	.RegTcxo 			= 0x4B,
	.RegPaDac 			= 0x4D,
	.RegFormerTemp 		= 0x5B,
	.RegBitRateFrac 	= 0x5D,
	.RegAgcRef 			= 0x61,
	.RegAgcThresh1 		= 0x62,
	.RegAgcThresh2 		= 0x63,
	.RegAgcThresh3 		= 0x64,
	.RegPll 			= 0x70,

};



void semtech_burst_write(SPIDriver * spip, uint8_t addr, uint8_t * data, uint8_t bufSize)
{
	//Will send the array of data over the SPI bus, programming each register
	//Starting from addr, and incrementing.

	uint8_t addrSend = 0b10000000 | addr; //Set MSB to 1 so for write mode

	spiSelect(spip);

	//Send the address, and then send the data
	spiStartSend(spip, 1, &addrSend);
	//Block until send complete
	while((*spip).state != SPI_READY) {}
	
	spiStartSend(spip, bufSize, &data);
	//Block until send complete
	while((*spip).state != SPI_READY) {}

	spiUnselect(spip);

}

void semtech_write(SPIDriver * spip, uint8_t addr, uint8_t val)
{
	//Will send the array of data over the SPI bus, programming each register
	//Starting from addr, and incrementing.

	uint8_t addrSend = 0b10000000 | addr; //Set MSB to 1 so for write mode

	spiSelect(spip);

	//Send the address, and then send the data
	spiStartSend(spip, 1, &addrSend);
	//Block until send complete
	while((*spip).state != SPI_READY) {}
	
	spiStartSend(spip, bufSize, &val);
	//Block until send complete
	while((*spip).state != SPI_READY) {}

	spiUnselect(spip);

}
void semtech_burst_read(SPIDriver * spip, uint8_t addr, uint8_t * datrcvptr, uint8_t bufLength)
{
	//Reads a series of registers from the semtech
	
	spiSelect(spip);

	spiStartSend(spip, 1, &addr);
	//Wait for send to complete
	while((*spip).state != SPI_READY) { }

	
	spiStartReceive(spip, bufLength, datrcvptr);
	while((*spip).state != SPI_READY) { }
	
	spiUnselect(spip);

}

void semtech_test_read(SPIDriver * spip,uint8_t baseAddr)
{
	
	//Will request the values of some registers, and write the output to serial.
	chprintf(DEBUG_CHP, "\r\nTesting SPI Connection...\r\n");
	
	uint8_t rcvData[12];
    uint8_t i;
    semtech_burst_read(spip, baseAddr, &rcvData, sizeof(rcvData));

    
    chprintf(DEBUG_CHP, "\r\n Reg values: \r\n" );
    for (i = 0; i<12; i++){
       
        chprintf(DEBUG_CHP, "\r\n %x %x\r\n ",i + baseAddr, rcvData[i]);
    };
	
}

void semtech_test_rcv(SPIDriver * spip){
	//Configures receive parameters

	semtech_write(spip, transceiver.RegRxBw, 0x100);

}
void semtech_transmit_data(SPIDriver * spip, uint8_t * data)
{
	//Sends data to the FIFO for transmission

	//TODO: Set semtech to tx mode if needed. This clears the FIFO.

	//TODO: Check if FIFO is full (handle excess data if needed)
	//There are 64 bytes in the FIFO (pg. 40)


	//Send data to FIFO
	semtech_burst_write(spip, 0x0, data, 0);

}


