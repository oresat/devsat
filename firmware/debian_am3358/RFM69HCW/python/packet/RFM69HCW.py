#!/usr/bin/env python
# file: DUT.py

"""
Experiment with the sx1231/RFM69HCW module

"""

import sys
import random
import threading
import time
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.SPI import SPI


# SX1231 Registers
sx1231_reg = {
    "RegFifo"           : 0x00,
    "RegOpMode"         : 0x01,
    "RegDataModul"      : 0x02,
    "RegBitrateMsb"     : 0x03,
    "RegBitrateLsb"     : 0x04,
    "RegFdevMsb"        : 0x05,
    "RegFdevLsb"        : 0x06,
    "RegFrfMsb"         : 0x07,
    "RegFrfMid"         : 0x08,
    "RegFrfLsb"         : 0x09,
    "RegOsc1"           : 0x0A,
    "RegAfcCtrl"        : 0x0B,
    "RegLowBat"         : 0x0C,
    "RegListen1"        : 0x0D,
    "RegListen2"        : 0x0E,
    "RegListen3"        : 0x0F,
    "RegVersion"        : 0x10,
    "RegPaLevel"        : 0x11,
    "RegPaRamp"         : 0x12,
    "RegOcp"            : 0x13,
    "Reserved14"        : 0x14,
    "Reserved15"        : 0x15,
    "Reserved16"        : 0x16,
    "Reserved17"        : 0x17,
    "RegLna"            : 0x18,
    "RegRxBw"           : 0x19,
    "RegAfcBw"          : 0x1A,
    "RegOokPeak"        : 0x1B,
    "RegOokAvg"         : 0x1C,
    "RegOokFix"         : 0x1D,
    "RegAfcFei"         : 0x1E,
    "RegAfcMsb"         : 0x1F,
    "RegAfcLsb"         : 0x20,
    "RegFeiMsb"         : 0x21,
    "RegFeiLsb"         : 0x22,
    "RegRssiConfig"     : 0x23,
    "RegRssiValue"      : 0x24,
    "RegDioMapping1"    : 0x25,
    "RegDioMapping2"    : 0x26,
    "RegIrqFlags2"      : 0x28,
    "RegRssiThresh"     : 0x29,
    "RegRxTimeout2"     : 0x2B,
    "RegPreambleMsb"    : 0x2C,
    "RegPreambleLsb"    : 0x2D,
    "RegSyncConfig"     : 0x2E,
    "RegSyncValue1"     : 0x2F,
    "RegSyncValue2"     : 0x30,
    "RegSyncValue3"     : 0x31,
    "RegSyncValue4"     : 0x32,
    "RegSyncValue5"     : 0x33,
    "RegSyncValue6"     : 0x34,
    "RegSyncValue7"     : 0x35,
    "RegSyncValue8"     : 0x36,
    "RegPacketConfig1"  : 0x37,
    "RegPayloadLength"  : 0x38,
    "RegNodeAdrs"       : 0x39,
    "RegBroadcastAdrs"  : 0x3A,
    "RegAutoModes"      : 0x3B,
    "RegFifoThresh"     : 0x3C,
    "RegPacketConfig2"  : 0x3D,
    "RegAesKey1"        : 0x3E,
    "RegAesKey2"        : 0x3F,
    "RegAesKey3"        : 0x40,
    "RegAesKey4"        : 0x41,
    "RegAesKey5"        : 0x42,
    "RegAesKey6"        : 0x43,
    "RegAesKey7"        : 0x44,
    "RegAesKey8"        : 0x45,
    "RegAesKey9"        : 0x46,
    "RegAesKey10"       : 0x47,
    "RegAesKey11"       : 0x48,
    "RegAesKey12"       : 0x49,
    "RegAesKey13"       : 0x4A,
    "RegAesKey14"       : 0x4B,
    "RegAesKey15"       : 0x4C,
    "RegAesKey16"       : 0x4D,
    "RegTemp1"          : 0x4E,
    "RegTemp2"          : 0x4F,
    "RegTestLna"        : 0x58,
    "RegTestTcxo"       : 0x59,
    "RegTestllBw"       : 0x5F,
    "RegTestDagc"       : 0x6F
}

# inverse dictionary for register name lookup
inv_sx1231_reg = {v: k for k, v in sx1231_reg.items()}

# sx1231 RegOpMode
# sx1231 Datasheet p 65
OPMODE_SEQUENCER_ON                =   (0b0  <<7)
OPMODE_SEQUENCER_OFF               =   (0b1  <<7)
OPMODE_LISTEN_ON                   =   (0b1  <<6)
OPMODE_LISTEN_OFF                  =   (0b0  <<6)
OPMODE_LISTEN_ABORT                =   (0b1  <<5)
OPMODE_SLEEP                      =   (0b000<<2)
OPMODE_STANDBY                    =   (0b001<<2)
OPMODE_FS                         =   (0b010<<2)
OPMODE_TRANSMITTER                =   (0b011<<2)
OPMODE_RECEIVER                   =   (0b100<<2)

# RegDataModul
DATAMODUL_Packet            =   (0b00     << 5)
DATAMODUL_Continuous        =   (0b10     << 5)
DATAMODUL_ContinuousNoSync  =   (0b11     << 5)
DATAMODUL_FSK               =   (0b00     << 3)
DATAMODUL_OOK               =   (0b01     << 3)
DATAMODUL_NoShaping         =   (0b00     << 0)

# RegPaLevel
PA0                         =   (0b1      << 7)
PA1                         =   (0b1      << 6)
PA2                         =   (0b1      << 5)

# RegAfcFei
AfcAutoOn                   =   (1 << 2)
AfcAutoclearOn              =   (1 << 3)
AfcDone                     =   (1 << 4)

#Automodes
EnterNone                   =   (0b000 << 5)
EnterFifoNotEmpty           =   (0b001 << 5)
EnterFifoLevel              =   (0b010 << 5)
EnterCrcOk                  =   (0b011 << 5)
EnterPayloadReady           =   (0b100 << 5)
EnterSyncAddress            =   (0b101 << 5)
EnterPacketSent             =   (0b110 << 5)
EnterFifoEmpty              =   (0b111 << 5)
ExitNone                    =   (0b000 << 2)
ExitFifoEmpty               =   (0b001 << 2)
ExitFifoLevel               =   (0b010 << 2)
ExitCrcOk                   =   (0b011 << 2)
ExitPayloadReady            =   (0b100 << 2)
ExitSyncAddress             =   (0b101 << 2)
ExitPacketSent              =   (0b110 << 2)
ExitTimeout                 =   (0b111 << 2)
InterSleep                  =   (0b00  << 0)
InterStdby                  =   (0b01  << 0)
InterRX                     =   (0b10  << 0)
InterTX                     =   (0b11  << 0)

# Modes
MODE_RX                     =   (1     << 4)
MODE_TX                     =   (3     << 2)



# DioMapping1
DIO_0_POS                   =   6
DIO_1_POS                   =   4
DIO_2_POS                   =   2
DIO_3_POS                   =   0
DIO1_RX_TIMEOUT            =   (0b11)


# DioMapping2
DIO_4_POS                   =   6
DIO_5_POS                   =   4
DIO_CLKOFF                  =   0b111
DIO_CLK_DIV32               =   0b101

DIO4_RXRDY                  =   (0b10)


# PA
PA0                         =   (1     << 7)
PA1                         =   (1     << 6)
PA2                         =   (1     << 5)

# RegLna
LnaZin50_AGC                =   (0x08)

# RegTestPllBW
PLLBandwidth_75kHz          =   (0x0   << 2)


# RegSyncConfig
SYNCCFG_SYNC_ON             =   (0b1<<7)
SYNCCFG_SYNC_OFF            =   (0b0<<7)
SYNCCFG_FILL_FIFO_INTR      =   (0b0<<6)
SYNCCFG_SIZE_2              =   (0b001<<3)
SYNCCFG_SIZE_3              =   (0b010<<3)
SYNCCFG_SIZE_4              =   (0b011<<3)


# Hardwired choices to bbb shield
G0_PIN                      =   "P9_12"
G1_PIN                      =   "P8_7"   # DIO1/DCLK
G2_PIN                      =   "P8_8"   # DIO2/DATA
G3_PIN                      =   "P8_9"
G4_PIN                      =   "P8_10"
G5_PIN                      =   "P8_12"
BLUE_LEDPIN                 =   "P9_41"
MODULE_EN                   =   "P9_23"
MODULE_RST                  =   "P9_15"
SPI0_MISO                   =   "P9_21"
SPI0_MOSI                   =   "P9_18"
SPI0_CLK                    =   "P9_22"
SPI0_CS                     =   "P9_17"

#initial defaults
default_LED_STATE           =   False
default_Fxosc               =   32e6
default_Fstep               =   61.03515625
default_callsign            =   None
                
default_node_id             = 0x33
default_network_id          = 1

default_carrier_freq        =   436500000
default_carrier_dev         =   20000
default_bitrate             =   1200

class CheckError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
      return repr(self.value)

class NoCallSign(Exception):
  def __init__(self, value):
        self.value = value
  def __str__(self):
      return repr(self.value)

"""
 Interrupt callbacks
"""
def g0int(a):
    print "g0"

def g1int(a):
    global count
    global byte
    rxrdy = GPIO.input(G4_PIN)
    if rxrdy == 1 :
        count = count+1
        value = GPIO.input(G2_PIN)
        if value == 1 :
            byte = byte | 1<<count
        else:
            byte = byte | 0<<count  #  no effect - conceptual

    if count >= 7:
        print hex(byte)," ",
        count = 0
        byte  = 0

    sys.stdout.flush()

def g2int(a):
    pass
# global count
# # print "g2"
# count = count+1

def g3int(a):
    print "rssi g3"

def g4int(a):
    print "rx_rdy g4"

def g5int(a):
    print "g5"

### END INTERRUPT CALLBACKS
##########################
"""
Example Usage:
    try:
        BEACON       = RFM69HCW(callsign="ABXCDE")
        # BEACON.start_tx()
        BEACON.report_setup()
        BEACON.blue_blink()
 
    except: 
        pass
"""
class RFM69HCW():
    def __init__(self, LED_STATE=default_LED_STATE, 
                 Fxosc=default_Fxosc,
                 Fstep=default_Fstep,
                 callsign=None,
                 node_id=default_node_id,
                 network_id=default_network_id,
                 carrier_freq=default_carrier_freq,
                 carrier_dev=default_carrier_dev,
                 carrier_bitrate=default_bitrate
                ):
        self.LED_STATE      = LED_STATE
        self.Fxosc          = Fxosc
        self.Fstep          = Fstep
        self.callsign       = callsign
        self.RFM_SPI        = SPI(0,0)
        self.RFM_SPI.msh    = 5000000
        self.carrier_freq   = carrier_freq
        self.carrier_dev    = carrier_dev
        self.bitrate        = carrier_bitrate
        self.node_id        = node_id
        self.network_id     = network_id

        if self.callsign is None:
            raise NoCallSign("FCC Callsign not defined")
        self.ord_callsign   = map(ord,list(self.callsign))
        self._io_setup()
        self.reset_radio()
        GPIO.output(BLUE_LEDPIN,GPIO.LOW)
        return

    def _io_setup(self):
        GPIO.setup(BLUE_LEDPIN, GPIO.OUT)
        GPIO.setup(MODULE_EN, GPIO.OUT)
        GPIO.setup(MODULE_RST, GPIO.OUT)
        GPIO.setup(G0_PIN, GPIO.IN)
        GPIO.setup(G1_PIN, GPIO.OUT)
        GPIO.setup(G2_PIN, GPIO.OUT)
        # GPIO.add_event_detect(G0_PIN, GPIO.FALLING, callback=g0int)
        # GPIO.add_event_detect(G0_PIN, GPIO.RISING,  callback=g0int)

    def _check_register(self, addr, value):
        vals = self.RFM_SPI.xfer2([addr, 0x0])
        if vals[1] != value:
            str = "addr: "+ hex(addr) + "(" + inv_sx1231_reg[addr] + ")" + " should be: " + hex(value) + " got: " + hex(vals[1])
            raise CheckError(str)
        print "Reg{",hex(addr),"}(",inv_sx1231_reg[addr],")\t\t=", hex(vals[1])

    def write_register(self, reg, val, checkit=False):
      addr=reg
      reg = reg | 0x80
      # print "reg is: ", bin(reg)
      # print "val is: " , bin(val)
      # RFM_SPI.writebytes([reg, val])
      self.RFM_SPI.xfer2([reg, val])
      if checkit==True:
         self._check_register(addr, val)
      return
    
    def read_register(self, reg):
      regval = self.RFM_SPI.xfer2([reg, 0x0])
      return regval[1]

    def reset_radio(self):
        self.blue_blink(2)
        GPIO.output(MODULE_EN,GPIO.HIGH)
        GPIO.output(MODULE_RST,GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(MODULE_RST,GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(MODULE_RST,GPIO.LOW)
        time.sleep(0.5)

    def blue_invert(self):
        if(self.LED_STATE) == True:
            self.blue_off()
        else:
            self.blue_on()

    def blue_off(self):
        self.LED_STATE=False
        GPIO.output(BLUE_LEDPIN,GPIO.HIGH)
        return

    def blue_on(self):
        self.LED_STATE=True
        GPIO.output(BLUE_LEDPIN,GPIO.LOW)
        return

    def blue_blink(self,n=3):
        for num in range(0,n*2):
            self.blue_invert()
            time.sleep(0.25)
        return

    def report_setup(self):
        print 'LED_STATE is:\t', self.LED_STATE
        print 'Fxosc is:    \t', self.Fxosc
        print 'Fstep is:    \t', self.Fstep
        print 'Callsign is: \t', self.callsign
        return

    # Facts:
    #   Fxosc = 32Mhz
    #   Fstep = 32e6/2^9  =  61.03515625
    #   Frf   = int(carrier_hz/Fstep)
    def write_carrier_freq(self, carrier_hz=436500000):
        frf      = int(carrier_hz / self.Fstep)
    
        # vals = RFM_SPI.xfer2([RegFrfMsb, 0x0, 0x0, 0x0])
        # print "Pre: vals=\t", hex(vals[0]), "\t", hex(vals[1]), "\t", hex(vals[2]), "\t", hex(vals[3])
    
        frfmsb = (frf>>16) & 0xff
        frfmid = (frf>>8)  & 0xff
        frflsb = frf       & 0xff
    
        wbuf      = [(sx1231_reg["RegFrfMsb"]|0x80), int(frfmsb), int(frfmid), int(frflsb)]
        self.RFM_SPI.writebytes(wbuf)
    
        vals = self.RFM_SPI.xfer2([sx1231_reg["RegFrfMsb"], 0x0, 0x0, 0x0])
        # print "Post: vals=\t", hex(vals[0]), "\t", hex(vals[1]), "\t", hex(vals[2]), "\t", hex(vals[3])
        return
    
    def set_freq_deviation(self, freq_dev_hz=20000):
        freqdev = int(freq_dev_hz/self.Fstep)
    
        wbuf    = [(sx1231_reg["RegFdevMsb"]|0x80), (int(freqdev>>8) & 0x3f), int(freqdev&0xff)]
        self.RFM_SPI.writebytes(wbuf)
        # print "fdev_msb:\t",
        # check_register(sx1231_reg["RegFdevMsb"], (int(freqdev>>8) & 0x3f))
        # print "\nfdev_lsb:\t",
        # check_register(sx1231_reg["RegFdevLsb"], (int(freqdev & 0xff)))
        # print "\n"
        return
    
    def set_bitrate(self, bitrate_hz=1200):
        rate = int(self.Fxosc/bitrate_hz)
    
        wbuf    = [(sx1231_reg["RegBitrateMsb"]|0x80), (int(rate>>8) & 0xff), int(rate&0xff)]
        self.RFM_SPI.writebytes(wbuf)
    
    def set_sync_value(fourbytelist):
        wbuf    = [(sx1231_reg["RegSyncValue1"]|0x80)] + fourbytelist
        self.RFM_SPI.writebytes(wbuf)
    
    def set_preamble(twobytelist):
        wbuf    = [(sx1231_reg["RegPreambleMsb"]|0x80)] + twobytelist
        self.RFM_SPI.writebytes(wbuf)

    """
    Experiment with automodes 
    """
    def config_rx_packet(self, pa, node_id=0x33, network_id=0x77):
        # Begin with sequencer on, listen off, and in standby
        self.write_register(sx1231_reg["RegOpMode"], OPMODE_SEQUENCER_ON|OPMODE_LISTEN_OFF|OPMODE_STANDBY, True) 

        # Packet Mode, FSK, No Shaping
        self.write_register(sx1231_reg["RegDataModul"], DATAMODUL_Packet|DATAMODUL_FSK|DATAMODUL_NoShaping) 

        self.write_carrier_freq(self.carrier_freq)
        self.set_freq_deviation(self.carrier_dev)
        self.set_bitrate(self.bitrate)

        # PA Output Power
        self.write_register(sx1231_reg["RegPaLevel"], pa )


        # DIO Mappings
                                                #  (DccFreq|RxBwMant|RxBwExp) Table 13
        self.write_register(sx1231_reg["RegRxBw"], (010<<5|0x10<<3|100<<0) ) # 20.8kHz?

        self.write_register(sx1231_reg["RegDioMapping1"], DIO1_RX_TIMEOUT<<DIO_1_POS            , True  )

        self.write_register(sx1231_reg["RegDioMapping2"], (DIO4_RXRDY<<DIO_4_POS | DIO_CLK_DIV32), True )

        # RSSI Thresh
        self.write_register(sx1231_reg["RegRssiThresh"], 0xdc, True)   # -220/2 = -110dBm?

        # Preamble length (0xaa..N)
        self.write_register(sx1231_reg["RegPreambleLsb"], 0xf, True)

        # Sync Config
        self.write_register(sx1231_reg["RegSyncConfig"], SYNCCFG_SYNC_ON | SYNCCFG_FILL_FIFO_INTR | SYNCCFG_SIZE_2, True)

        # Sync Word
        self.write_register(sx1231_reg["RegSyncValue1"], node_id   , True )
        self.write_register(sx1231_reg["RegSyncValue2"], network_id, True )
        
        # Packet config here

        return
    
    def start_tx_packet(self):
        return

    def start_rx_packet(self):
        return

    def stop(self):
        return


def PAOutputCfg(PA, Power):
    return (((PA) & (PA0 | PA1 | PA2)) | ((Power) & 0x1F))

if __name__ == "__main__":
    try:
        BEACON       = RFM69HCW(callsign="KG7EYD")
        # BEACON.start_tx()
        BEACON.report_setup()
        BEACON.blue_blink()
        BEACON.config_rx_packet(PAOutputCfg(PA0, 0x1F))
        # Too much power? 
        # BEACON.config_rx_packet(PAOutputCfg(PA1, 0x1F))
        # BEACON.config_rx_packet(PAOutputCfg(PA2|PA1, 0x1F))


        BEACON.reset_radio()
        # BEACON.stop()
        print("Bye.")

    except NoCallSign as e:
        print ('No FCC Call Sign Entered', e.value)

    except CheckError as e:
        print ('Check Error', e.value)

    except KeyboardInterrupt:
        BEACON.stop()
        u.info("\r\nQuitting-keyboard interrupt.")

