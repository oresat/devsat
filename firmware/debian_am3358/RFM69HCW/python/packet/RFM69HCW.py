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

# PA
PA0                         =   (1     << 7)
PA1                         =   (1     << 6)
PA2                         =   (1     << 5)

# RegLna
LnaZin50_AGC                =   (0x08)

# RegTestPllBW
PLLBandwidth_75kHz          =   (0x0   << 2)

# RegDataModul
DataModul_Packet            =   (0     << 5)
DataModul_Continuous        =   (2     << 5)
DataModul_ContinuousNoSync  =   (3     << 5)
DataModul_FSK               =   (0     << 3)
DataModul_OOK               =   (1     << 3)
DataModul_NoShaping         =   (0     << 0)

# sx1231 RegOpMode s
# sx1231 Datasheet p 65
SLEEP_MODE                  =   (0b000<<2)
STANDBY_MODE                =   (0b001<<2)
FS_MODE                     =   (0b010<<2)
TRANSMITTER_MODE            =   (0b011<<2)
RECEIVER_MODE               =   (0b100<<2)

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
default_callsign            = None

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

class RFM69HCW():
    def __init__(self, LED_STATE=default_LED_STATE, Fxosc=default_Fxosc, Fstep=default_Fstep,  callsign=None):
        self.LED_STATE      = LED_STATE
        self.Fxosc          = Fxosc
        self.Fstep          = Fstep
        self.callsign       = callsign
        if self.callsign is None:
            raise NoCallSign("FCC Callsign not defined")
        self.ord_callsign   = map(ord,list(self.callsign))
        self._io_setup()
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

    def blue_invert(self):
      if(self.LED_STATE) == 1:
        self.LED_STATE=0
        GPIO.output(BLUE_LEDPIN,GPIO.HIGH)
      else:
        self.LED_STATE=1
        GPIO.output(BLUE_LEDPIN,GPIO.LOW)
      return
    
    def blue_blink(self,n=3):
        for num in range(0,n*2):
            self.blue_invert()
            time.sleep(0.25)
        return

    def report_setup(self):
        print 'LED_STATE is: ', self.LED_STATE
        print 'Fxosc is: ', self.Fxosc
        print 'Fstep is: ', self.Fstep
        print 'Callsign is: ', self.callsign
        return

    def start_tx(self):
        return

    def start_rx(self):
        return

    def stop(self):
        return


if __name__ == "__main__":
    try:
       BEACON       = RFM69HCW(callsign="KG7EYD")
       # BEACON.start_tx()
       BEACON.report_setup()
       BEACON.blue_blink(10)

       # BEACON.stop()
       print("Bye.")

    except NoCallSign as e:
      print ('No FCC Call Sign Entered', e.value)

    except CheckError as e:
      print ('Check Error', e.value)

    except KeyboardInterrupt:
        BEACON.stop()
        u.info("\r\nQuitting-keyboard interrupt.")





