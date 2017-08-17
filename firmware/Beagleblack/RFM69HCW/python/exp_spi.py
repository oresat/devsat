#!/usr/bin/python


import time
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.SPI import SPI

# Registers
RegFifo          = 0x00
RegOpMode        = 0x01
RegDataModul     = 0x02
RegBitrateMsb    = 0x03
RegFrfMsb        = 0x07
RegFrfMid        = 0x08
RegFrfLsb        = 0x09
RegAfcBw         = 0x1a


# sx1231 RegOpMode s
#   sx1231 Datasheet p 65
SLEEP_MODE       = 0b000
STANDBY_MODE     = 0b001
FS_MODE          = 0b010
TRANSMITTER_MODE = 0b011
RECEIVER_MODE    = 0b100



BLUE_LEDPIN = "P9_41"
MODULE_EN   = "P9_23"
MODULE_RST  = "P9_15"
SPI0_MISO   = "P9_21"
SPI0_MOSI   = "P9_18"
SPI0_CLK    = "P9_22"
SPI0_CS     = "P9_17"

LED_STATE   = 1

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
                GPIO.output(BLUE_LEDPIN, GPIO.HIGH)
                time.sleep(0.25)
                GPIO.output(BLUE_LEDPIN, GPIO.LOW)
                time.sleep(0.25)

def RFM69HCW_Write_Register(reg, val):
    reg = reg | 0x80
    print "reg is: ", bin(reg)
    print "val is: " , bin(val)
    # RFM_SPI.writebytes([reg, val])
    RFM_SPI.xfer2([reg, val])

def RFM69HCW_Read_Register(reg):
    regval = RFM_SPI.xfer2([reg, 0x0])
    print "current_regval is:\t", hex(reg), "\t",  hex(regval[1])
    return regval[1]

# Fxosc = 32Mhz
# Fstep = 32e6/2^9  =  61.03515625
# Frf   = int(carrier_hz/Fstep)
def RFM69HCW_Write_Carrier_Freq(carrier_hz):
    Fstep    = 61.03515625

    frf      = int(carrier_hz / Fstep) 
    
    # vals = RFM_SPI.xfer2([RegFrfMsb, 0x0, 0x0, 0x0])
    # print "Pre: vals=\t", hex(vals[0]), "\t", hex(vals[1]), "\t", hex(vals[2]), "\t", hex(vals[3])
    
    frfmsb = (frf>>16) & 0xff
    frfmid = (frf>>8)  & 0xff
    frflsb = frf       & 0xff
    
    wbuf      = [(RegFrfMsb|0x80), int(frfmsb), int(frfmid), int(frflsb)]
    RFM_SPI.writebytes(wbuf)
    
    vals = RFM_SPI.xfer2([RegFrfMsb, 0x0, 0x0, 0x0])
    print "Post: vals=\t", hex(vals[0]), "\t", hex(vals[1]), "\t", hex(vals[2]), "\t", hex(vals[3])


def RFM69HCW_config_xcvr():
    # Check register values 
    # readit = RFM69HCW_Read_Register(RegAfcBw)
    # readit = RFM69HCW_Read_Register(RegOpMode)

    # RegOpMode 
    #    set mode FS - Frequency Synthesizer mode
    RFM69HCW_Write_Register(RegOpMode, FS_MODE<<2)  
    time.sleep(0.05)

    # Set Carrier Frequency
    RFM69HCW_Write_Carrier_Freq(436500000)





    RFM69HCW_Write_Register(RegAfcBw, 0x8b)  
    time.sleep(0.05)
 
def spi_config():
    global RFM_SPI
    GPIO.output(MODULE_EN,GPIO.HIGH)
    GPIO.output(MODULE_RST,GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(MODULE_RST,GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(MODULE_RST,GPIO.LOW)
    time.sleep(0.5)
    RFM_SPI     = SPI(0,0)
    RFM_SPI.msh = 5000000

# 0x1A RegAfcBw 0x8A 0x8B Channel Filter BW control during the AFC routine
def spi_test():
#       global RFM_SPI
        while True:
                blue_invert()
                # RegAfcBw = RFM69HCW_Read_Register(0x1A)
                while True:
                    time.sleep(0.75)
                    RFM69HCW_config_xcvr()

if __name__ == "__main__":
    try:
        io_setup()
        #blue_blink(3)
        spi_config()
        spi_test()
        GPIO.cleanup()
        

    except KeyboardInterrupt:
        GPIO.cleanup()
        print ("\nQuitting-Bye!")

