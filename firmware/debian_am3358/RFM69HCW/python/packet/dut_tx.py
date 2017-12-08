#!/usr/bin/env python

# Test TX beacon

import RFM69HCW
import time
import Utils as u

try:
    DUT_TX       = RFM69HCW.RFM69HCW(callsign="KG7EYD")

    DUT_TX.report_setup()
    DUT_TX.config_packet(RFM69HCW.PAOutputCfg(RFM69HCW.PA0, 0x1F))

    # Experiment with different power levels
    # DUT_TX.config_packet(RFM69HCW.PAOutputCfg(RFM69HCW.PA1, 0x1F))
    # DUT_TX.config_packet(RFM69HCW.PAOutputCfg(RFM69HCW.PA2|RFM69HCW.PA1, 0x1F))

    # Test reading RSSI
    print "RSSI:\t",DUT_TX.RSSI()
    while True:
        DUT_TX.send([10,9,8,7,6,5,4,3,2,1,0])
        time.sleep(1.5)
        print "."

    DUT_TX.reset_radio()
    print("Bye.")

except RFM69HCW.NoCallSign as e:
    print ('No FCC Call Sign Entered', e.value)

except RFM69HCW.CheckError as e:
    print ('Check Error', e.value)

except KeyboardInterrupt:
    u.info("Reset radio...")
    DUT_TX.reset_radio()
    u.info("Quitting-keyboard interrupt.")

