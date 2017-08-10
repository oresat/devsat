#!/usr/bin/python


import time
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_BBIO.SPI import SPI

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

# 0x1A RegAfcBw 0x8A 0x8B Channel Filter BW control during the AFC routine
def spi_test():
#	global RFM_SPI
	GPIO.output(MODULE_EN,GPIO.HIGH)
	GPIO.output(MODULE_RST,GPIO.LOW)
	time.sleep(0.5)
	GPIO.output(MODULE_RST,GPIO.HIGH)
	time.sleep(0.5)
	GPIO.output(MODULE_RST,GPIO.LOW)
	time.sleep(0.5)
	RFM_SPI     = SPI(0,0)
	RFM_SPI.msh=500000
	while True:
		blue_invert()
		#msh         = RFM_SPI.msh
		#print "msh=",msh

		RFM_SPI.writebytes([0x1A, 0x0])
		time.sleep(1)

		RegAfcBw = RFM_SPI.xfer2([0x1A, 0x0])
#		print "RegAfcBw=", format(RegAfcBw, '02x')
		print "RegAfcBw=", hex(RegAfcBw[1])

if __name__ == "__main__":
    try:
	io_setup()
	#blue_blink(3)
	spi_test()
	GPIO.cleanup()
	

    except KeyboardInterrupt:
	GPIO.cleanup()
    	print ("\nQuitting")

