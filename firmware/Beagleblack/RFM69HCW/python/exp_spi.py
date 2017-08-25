#!/usr/bin/python

import time
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.SPI import SPI

class ModeError(Exception):
  def __init__(self, value):
    self.value = value
  def __str__(self):
    return repr(self.value)

class NoCallSign(Exception):
  def __init__(self, value):
    self.value = value
  def __str__(self):
    return repr(self.value)

class CheckError(Exception):
  def __init__(self, value):
    self.value = value
  def __str__(self):
    return repr(self.value)


# Registers
sx1231_reg = {
    "RegFifo"          : 0x00,
    "RegOpMode"        : 0x01,
    "RegDataModul"     : 0x02,
    "RegBitrateMsb"    : 0x03,
    "RegBitrateLsb"    : 0x04,
    "RegFdevMsb"       : 0x05,
    "RegFdevLsb"       : 0x06,
    "RegFrfMsb"        : 0x07,
    "RegFrfMid"        : 0x08,
    "RegFrfLsb"        : 0x09,
    "RegPaLevel"       : 0x11,
    "RegLna"           : 0x18,
    "RegRxBw"          : 0x19,
    "RegAfcBw"         : 0x1a,
    "RegAfcFei"        : 0x1e,
    "RegRssiThresh"    : 0x29,
    "RegPreambleMsb"   : 0x2c,
    "RegSyncConfig"    : 0x2e,
    "RegSyncValue1"    : 0x2f,
    "RegPacketConfig1" : 0x37,
    "RegPayloadLength" : 0x38,
    "RegAutoModes"     : 0x3b,
    "RegFifoThresh"    : 0x3c,
    "RegTestPllBW"     : 0x5f,
    }

# inverse dictionary for register name lookup
inv_sx1231_reg = {v: k for k, v in sx1231_reg.items()}

# RegSyncConfig
SyncOn              =  (1 << 7)
FifoFillSyncAddress =  (0 << 6)
FifoFillCondition   =  (1 << 6)

def SyncSize(bytes):
  return ((((bytes) - 1) & 0x7) << 3)

def SyncTol(errors):
  return ((errors) & 0x7)


# RegAfcFei
AfcAutoOn           = (1 << 2)
AfcAutoclearOn      = (1 << 3)


#Automodes
EnterNone           =   (0b000 << 5)
EnterFifoNotEmpty   =   (0b001 << 5)
EnterFifoLevel      =   (0b010 << 5)
EnterCrcOk          =   (0b011 << 5)
EnterPayloadReady   =   (0b100 << 5)
EnterSyncAddress    =   (0b101 << 5)
EnterPacketSent     =   (0b110 << 5)
EnterFifoEmpty      =   (0b111 << 5)
ExitNone            =   (0b000 << 2)
ExitFifoEmpty       =   (0b001 << 2)
ExitFifoLevel       =   (0b010 << 2)
ExitCrcOk           =   (0b011 << 2)
ExitPayloadReady    =   (0b100 << 2)
ExitSyncAddress     =   (0b101 << 2)
ExitPacketSent      =   (0b110 << 2)
ExitTimeout         =   (0b111 << 2)
InterSleep          =   (0b00  << 0)
InterStdby          =   (0b01  << 0)
InterRX             =   (0b10  << 0)
InterTX             =   (0b11  << 0)

# Modes
MODE_RX                    = (1 << 4)
MODE_TX                    = (3 << 2)

# PA
PA0                        = (1 << 7)
PA1                        = (1 << 6)
PA2                        = (1 << 5)

# RegLna
LnaZin50_AGC               = (0x08)

# RegTestPllBW
PLLBandwidth_75kHz         = (0x0 << 2)

# RegDataModul
DataModul_Packet           = (0 << 5)
DataModul_Continuous       = (2 << 5)
DataModul_ContinuousNoSync = (3 << 5)
DataModul_FSK              = (0 << 3)
DataModul_OOK              = (1 << 3)
DataModul_NoShaping        = (0 << 0)

# sx1231 RegOpMode s
# sx1231 Datasheet p 65
SLEEP_MODE       = 0b000
STANDBY_MODE     = 0b001
FS_MODE          = 0b010
TRANSMITTER_MODE = 0b011
RECEIVER_MODE    = 0b100

BLUE_LEDPIN      = "P9_41"
MODULE_EN        = "P9_23"
MODULE_RST       = "P9_15"
SPI0_MISO        = "P9_21"
SPI0_MOSI        = "P9_18"
SPI0_CLK         = "P9_22"
SPI0_CS          = "P9_17"

LED_STATE        = 1

Fxosc            = 32e6
Fstep            = 61.03515625

def check_register(addr, value):
  vals = RFM_SPI.xfer2([addr, 0x0])
  if vals[1] != value:
    str = "addr: "+ hex(addr) + " should be: " +hex(value) + " got: " + hex(vals[1])
    raise CheckError(str)
  print "Reg{",hex(addr),"}(",inv_sx1231_reg[addr],")=", hex(vals[1])


def PAOutputCfg(PA, Power):
  return (((PA) & (PA0 | PA1 | PA2)) | ((Power) & 0x1F))

def io_setup():
  GPIO.setup(BLUE_LEDPIN, GPIO.OUT)
  GPIO.setup(MODULE_EN, GPIO.OUT)
  GPIO.setup(MODULE_RST, GPIO.OUT)

def blue_invert():
  global LED_STATE
  if(LED_STATE) == 1:
    LED_STATE=0
    GPIO.output(BLUE_LEDPIN,GPIO.HIGH)
  else:
    LED_STATE=1
    GPIO.output(BLUE_LEDPIN,GPIO.LOW)

def blue_blink(n):
  for num in range(0,n):
    blue_invert()
    time.sleep(0.25)

def RFM69HCW_Write_Register(reg, val):
  reg = reg | 0x80
  # print "reg is: ", bin(reg)
  # print "val is: " , bin(val)
  # RFM_SPI.writebytes([reg, val])
  RFM_SPI.xfer2([reg, val])

def RFM69HCW_Read_Register(reg):
  regval = RFM_SPI.xfer2([reg, 0x0])
  return regval[1]

# TODO: Collapse some of these multibyte register writes into a single def Wed 23 August 2017 17:08:45 (PDT)


# Facts:
#   Fxosc = 32Mhz
#   Fstep = 32e6/2^9  =  61.03515625
#   Frf   = int(carrier_hz/Fstep)
def RFM69HCW_Write_Carrier_Freq(carrier_hz):

  global Fstep 
  frf      = int(carrier_hz / Fstep)

  # vals = RFM_SPI.xfer2([RegFrfMsb, 0x0, 0x0, 0x0])
  # print "Pre: vals=\t", hex(vals[0]), "\t", hex(vals[1]), "\t", hex(vals[2]), "\t", hex(vals[3])

  frfmsb = (frf>>16) & 0xff
  frfmid = (frf>>8)  & 0xff
  frflsb = frf       & 0xff

  wbuf      = [(sx1231_reg["RegFrfMsb"]|0x80), int(frfmsb), int(frfmid), int(frflsb)]
  RFM_SPI.writebytes(wbuf)

  vals = RFM_SPI.xfer2([sx1231_reg["RegFrfMsb"], 0x0, 0x0, 0x0])
  # print "Post: vals=\t", hex(vals[0]), "\t", hex(vals[1]), "\t", hex(vals[2]), "\t", hex(vals[3])

def RFM69HCW_Set_Freq_Deviation(freq_dev_hz):
  global Fstep 
  freqdev = int(freq_dev_hz/Fstep)

  wbuf    = [(sx1231_reg["RegFdevMsb"]|0x80), (int(freqdev>>8) & 0x3f), int(freqdev&0xff)]
  RFM_SPI.writebytes(wbuf)

def RFM69HCW_Set_Bitrate(bitrate_hz):
  global Fxosc 
  rate = int(Fxosc/bitrate_hz)

  wbuf    = [(sx1231_reg["RegBitrateMsb"]|0x80), (int(rate>>8) & 0xff), int(rate&0xff)]
  RFM_SPI.writebytes(wbuf)


def RFM69HCW_Set_Sync_Value(fourbytelist):
  wbuf    = [(sx1231_reg["RegSyncValue1"]|0x80)] + fourbytelist 
  RFM_SPI.writebytes(wbuf)

def RFM69HCW_Set_Preamble(twobytelist):
  wbuf    = [(sx1231_reg["RegPreambleMsb"]|0x80)] + twobytelist 
  RFM_SPI.writebytes(wbuf)

def RFM69HCW_config_xcvr(OpMode, pa):
  # RegOpMode
  #    set mode FS - Frequency Synthesizer mode
  RFM69HCW_Write_Register(sx1231_reg["RegOpMode"], FS_MODE<<2)
  time.sleep(0.05)

  # # Set Carrier Frequency
  RFM69HCW_Write_Carrier_Freq(436500000)

  RFM69HCW_Set_Freq_Deviation(2500)

  RFM69HCW_Set_Bitrate(2400)

  RFM69HCW_Write_Register(sx1231_reg["RegDataModul"], DataModul_Packet | DataModul_FSK | DataModul_NoShaping)

  # # PLL Bandwith
  RFM69HCW_Write_Register(sx1231_reg["RegTestPllBW"], PLLBandwidth_75kHz )

  # # LNA Input Impedance
  RFM69HCW_Write_Register(sx1231_reg["RegLna"], LnaZin50_AGC)

  # # PA Output Power
  RFM69HCW_Write_Register(sx1231_reg["RegPaLevel"], pa )

  # Op Mode
  if OpMode == MODE_TX:
    autoModes = EnterFifoNotEmpty | InterTX | ExitPacketSent
  else: 
    raise ModeError("MODE_TX only at this time")

  RFM69HCW_Write_Register(sx1231_reg["RegAutoModes"], autoModes )
  check_register(sx1231_reg["RegAutoModes"], autoModes)

  RFM69HCW_Write_Register(sx1231_reg["RegRxBw"], 0x55 )
  check_register(sx1231_reg["RegRxBw"], 0x55)

  RFM69HCW_Write_Register(sx1231_reg["RegRssiThresh"], 0x70 )  # VERY Sensitive?
  check_register(sx1231_reg["RegRssiThresh"], 0x70)

  SyncConfig = SyncOn | FifoFillSyncAddress | SyncSize(1) | SyncTol(0)
  RFM69HCW_Write_Register(sx1231_reg["RegSyncConfig"], SyncConfig)
  check_register(sx1231_reg["RegSyncConfig"], SyncConfig)

  RFM69HCW_Set_Sync_Value([0xe7, 0xe7, 0xe7, 0xe7])

  RFM69HCW_Write_Register(sx1231_reg[sx1231_reg["RegPacketConfig1"], 0x08)
  check_register(sx1231_reg["RegPacketConfig1"], 0x08)

  RFM69HCW_Set_Preamble([0x00, 0x10])

"""
  	Sets the payload length

		The payload length needs to be equal to the buffer of data to be sent
		when the tx ready signal is produces on fifo_not_empty. If the tx
		ready signal is received from a fifo threshold reached condition
		then the payload length needs to be the same as the fifo threshold and
		the buffer needs to be one larger than the payload size.

		When using auto modes be sure to set the transceiver into standby mode
		it will wake and do its thing automagically.

"""

  RFM69HCW_Write_Register(sx1231_reg["RegPayloadLength"], 0x05)
  check_register(sx1231_reg["RegPayloadLength"], 0x05)


	# To trigger on a fifo threshhold set RegFifoThresh to PACKET_LENGTH
	# Trigger on fifo not empty 
  RFM69HCW_Write_Register(sx1231_reg["RegFifoThresh"], 0x04)
  check_register(sx1231_reg["RegFifoThresh"], 0x04)

  RFM69HCW_Write_Register(sx1231_reg["RegAfcFei"], AfcAutoOn | AfcAutoclearOn)
  check_register(sx1231_reg["RegAfcFei"], AfcAutoOn | AfcAutoclearOn)

  RFM69HCW_Write_Register(sx1231_reg["RegAfcBw"], 0x08)
  check_register(sx1231_reg["RegAfcBw"], 0x8b)

  RFM69HCW_Write_Register(sx1231_reg["RegOpMode"], OpMode)
  check_register(sx1231_reg["RegOpMode"], OpMode )


  time.sleep(0.05)

def spi_config():
  global RFM_SPI
  blue_blink(2)
  GPIO.output(MODULE_EN,GPIO.HIGH)
  GPIO.output(MODULE_RST,GPIO.LOW)
  time.sleep(0.5)
  GPIO.output(MODULE_RST,GPIO.HIGH)
  time.sleep(0.5)
  GPIO.output(MODULE_RST,GPIO.LOW)
  time.sleep(0.5)
  RFM_SPI     = SPI(0,0)
  RFM_SPI.msh = 5000000

def spi_test():
  RFM69HCW_config_xcvr(MODE_TX, PAOutputCfg(PA0, 0x0))

if __name__ == "__main__":
  try:
    io_setup()
    spi_config()
    spi_test()
    GPIO.output(BLUE_LEDPIN, GPIO.LOW)
    print "End. Ctl-C to Quit..."
    while True:
      blue_invert()
      time.sleep(0.5)


  except KeyboardInterrupt:
    GPIO.cleanup()
    print ("\nQuitting-Bye!")

  except ModeError as e:
    print ('Mode not supported:', e.value)

  except NoCallSign as e:
    print ('No FCC Call Sign Entered', e.value)

  except CheckError as e:
    print ('Check Error', e.value)

