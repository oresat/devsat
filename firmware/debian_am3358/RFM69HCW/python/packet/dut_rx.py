#!/usr/bin/env python

# Test RX from beacon

import RFM69HCW
import time
import Utils as u

try:
    DUT_RX       = RFM69HCW.RFM69HCW(callsign="KG7EYD")

    DUT_RX.report_setup()
    DUT_RX.config_packet(RFM69HCW.PAOutputCfg(RFM69HCW.PA0, 0x1F))

    # Experiment with different power levels
    # DUT_RX.config_packet(RFM69HCW.PAOutputCfg(RFM69HCW.PA1, 0x1F))
    # DUT_RX.config_packet(RFM69HCW.PAOutputCfg(RFM69HCW.PA2|RFM69HCW.PA1, 0x1F))

    # Test reading RSSI
    print "RSSI:\t",DUT_RX.RSSI()
    while True:
        # DUT_RX.send([1,2,3])
        time.sleep(0.5)
        DUT_RX.receive()
        if RFM69HCW.g0_flag==True:
            bytesrx = DUT_RX.read_fifo()
            print bytesrx

    DUT_RX.reset_radio()
    print("Bye.")

except RFM69HCW.NoCallSign as e:
    print ('No FCC Call Sign Entered', e.value)

except RFM69HCW.CheckError as e:
    print ('Check Error', e.value)

except KeyboardInterrupt:
    u.info("Reset radio...")
    DUT_RX.reset_radio()
    u.info("Quitting-keyboard interrupt.")

