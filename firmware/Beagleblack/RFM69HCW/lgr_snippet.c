// low-gain-radio/firmware/drivers/tranceiver.c

void configure_transceiver(uint8_t OpModeCfg, uint8_t RegPAOutputCfg){
	/* Change to frequency synthesizer mode */
	trans_write_register(transceiver.RegOpMode, (uint8_t[]){ModeFS}, 1);
	/* turn modulation to frequency shift keying */
	setCarrierFrequency(436500000);
	setFrequencyDeviation(2500);
	setBitrate(2400);

	trans_write_register(transceiver.RegDataModul, (uint8_t[]){Packet | FSK | NoShaping}, 1);
	/* adjust PLL bandwidth to 75kHz */
	trans_write_register(transceiver.RegTestPLL, (uint8_t[]){PLLBandwidth_75kHz}, 1);
	/* set LNA's input impedance to 50 ohm */
	trans_write_register(transceiver.RegLna, (uint8_t[]){LnaZin50}, 1);
	
	/* configure PA output power */
	trans_write_register(transceiver.RegPaLevel, (uint8_t[]){RegPAOutputCfg}, 1);

	uint8_t autoModes = 0;
	if(OpModeCfg == Mode_TX)
		autoModes = EnterFifoNotEmpty | InterTX | ExitPacketSent;
	else
		autoModes = EnterNone | InterSleep | ExitNone;
		//autoModes = EnterSyncAddress | InterRX | ExitFifoNotEmpty;
		//autoModes = EnterPayloadReady | InterSleep | ExitFifoEmpty;
	trans_write_register(transceiver.RegAutoModes, &autoModes, 1);

	trans_write_register(transceiver.RegRxBw, (uint8_t[]){0x55}, 1);
	trans_write_register(transceiver.RegRssiThresh, (uint8_t[]){0x70}, 1);

	uint8_t SyncConfig = SyncOn | FifoFillSyncAddress | SyncSize(1) | SyncTol(0);
	trans_write_register(transceiver.RegSyncConfig, &SyncConfig, 1);

	/*Sync word setup*/
	trans_write_register(transceiver.RegSyncValue1, (uint8_t[]){0xE7, 0xE7, 0xE7, 0xE7}, 4);

	/*Setup the packet config: no encoding no crc*/
	trans_write_register(transceiver.RegPacketConfig1, (uint8_t[]){0x08}, 1);

	//Sets preamble size
	trans_write_register(transceiver.RegPreambleMsb, (uint8_t[]){0x00, 0x10}, 2);

	//Sets the payload length
	/*
		The payload length needs to be equal to the buffer of data to be sent 
		when the tx ready signal is produces on fifo_not_empty. If the tx 
		ready signal is received from a fifo threshold reached condition
		then the payload length needs to be the same as the fifo threshold and 
		the buffer needs to be one larger than the payload size.

		When using auto modes be sure to set the transceiver into standby mode
		it will wake and do its thing automagically.

	*/
	trans_write_register(transceiver.RegPayloadLength, (uint8_t[]){5}, 1);

	/* To trigger on a fifo threshhold set RegFifoThresh to PACKET_LENGTH*/
	/* Trigger on fifo not empty */
	trans_write_register(transceiver.RegFifoThresh, (uint8_t[]){0x04}, 1);

	trans_write_register(transceiver.RegAfcFei, (uint8_t[]){AfcAutoOn | AfcAutoclearOn}, 1);

	/* Set transceiver mode */
	trans_write_register(transceiver.RegOpMode, (uint8_t[]){OpModeCfg}, 1);
}
