/*
    Code for communication with Semtech SX1236 using SPI and GPIO

    Written by Evan Yand et.al. for the Portland State Aerospace Society

    Licensed GPL v3 August 2017
*/

#include "ch.h"
#include "chprintf.h"
#include "board.h"
#include "hal.h"
#include "semtech-dev-board-registers.h"


#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

#define FXOSC (32000000) //32MHz
#define Fstep (FXOSC/(1<<19))
#define FstepMul ((uint64_t)(1 << 8))
#define FstepDiv ((uint64_t)(15625))


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

void semtech_write(SPIDriver * spip, uint8_t addr, uint8_t val, uint8_t bufSize)
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

uint8_t semtech_read(SPIDriver * spip, uint8_t addr){
	uint8_t val = 0;
	
	spiSelect(spip);

	spiStartSend(spip, 1, &addr);
	//Wait for send to complete
	while((*spip).state != SPI_READY) { }

	
	spiStartReceive(spip, 1, &val);
	while((*spip).state != SPI_READY) { }
	
	spiUnselect(spip);

	return(val);

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


void semtech_transmit_data(SPIDriver * spip, uint8_t * data)
{
	//Sends data to the FIFO for transmission

	//TODO: Set semtech to tx mode if needed. This clears the FIFO.

	//TODO: Check if FIFO is full (handle excess data if needed)
	//There are 64 bytes in the FIFO (pg. 40)


	//Send data to FIFO
	semtech_burst_write(spip, 0x0, data, 0);

}

void semtech_reset(void){
	
    palClearPort(GPIOA, 8);
    chThdSleep(100);
    palSetPort(GPIOA, 8);
}



void semtech_print_regs(SPIDriver * spip){
	/* Dumps all registers to serial out */
	
	chprintf(DEBUG_CHP, "--------------RegVals:---------------\r\n");

	#define X(SXreg) chprintf(DEBUG_CHP, "%s: \t\t\t %x\t Default:\t %x \r\n",\
	 #SXreg, semtech_read(spip, transceiver.SXreg), POR_defaults.SXreg);
	SEMTECH_REGISTERS
	#undef X

	chprintf(DEBUG_CHP, "\n\n\n");
}

void semtech_config(SPIDriver * spip){
	/* Programs SX1236 as defined in intended_setup
	*/
	
	#define X(SXreg) semtech_write(spip, transceiver.SXreg, defaults.SXreg, 1);
	SEMTECH_REGISTERS
	#undef X
}

uint8_t semtech_read_temp(SPIDriver * spip){
	//Reads the Tempenature value from the SX1236
	semtech_write(&SPID1, transceiver.RegOpMode, 0x08, 1);
    chThdSleepMilliseconds(1);
    semtech_write(&SPID1, transceiver.RegOpMode, 0x9, 1);
    chThdSleepMilliseconds(1);
    semtech_write(&SPID1, transceiver.RegOpMode, 0xc,1);
    chThdSleepMilliseconds(1);
    semtech_write(&SPID1, transceiver.RegImageCal, 0x3,1);
    chThdSleepMilliseconds(1);
    semtech_write(&SPID1, transceiver.RegImageCal, 0x2, 1);
    chThdSleepMilliseconds(1);
    semtech_write(&SPID1, transceiver.RegOpMode, 0x8, 1);

	return(semtech_read(spip, transceiver.RegTemp));
}
void semtech_listen(SPIDriver * spip) {
	/* Test fn to listen for packets, and dump them to uart
	*/
	uint8_t OpMode;

	//RMW OpMode
	OpMode = semtech_read(spip, transceiver.RegOpMode);
	OpMode = OpMode | 0b00000101;  
	OpMode = OpMode & 0b11111101;
	semtech_write(spip, transceiver.RegOpMode, OpMode, 1);


	while(1){
		while( (semtech_read(spip, transceiver.RegIrqFlags2) & 0b00000100) != 1) {}

		chprintf(DEBUG_CHP, "Preamble Detected! \r\n");

	}
}

void semtech_beacon(SPIDriver * spip, uint8_t payload){
	//semtech_write(spip, transceiver.RegFifo);
	(void)(payload);

	//Set Beacon Mode
	semtech_write(spip, transceiver.RegPacketConfig2, 0x48, 1);

	//Config Sequencer
	semtech_write(spip, transceiver.RegSeqConfig1, 0x04, 1);

	//Start Sequencer
	semtech_write(spip, transceiver.RegSeqConfig1, 0x84, 1);


}

void semtech_continuous(SPIDriver * spip){
	//Puts SX 1236 in continuous mode

	(void)(spip);

}
