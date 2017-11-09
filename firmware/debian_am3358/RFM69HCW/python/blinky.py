#!/usr/bin/python

# Blink the blue light on the protoboard

import time
import Adafruit_BBIO.GPIO as GPIO

def io_setup():
        GPIO.setup("P9_41", GPIO.OUT)


def blue_blink(n):
        for num in range(0,n):
                GPIO.output("P9_41", GPIO.HIGH)
                time.sleep(0.25)
                GPIO.output("P9_41", GPIO.LOW)
                time.sleep(0.25)


if __name__ == "__main__":
    try:
        io_setup()
        blue_blink(3)
        GPIO.cleanup()

    except KeyboardInterrupt:
        GPIO.cleanup()
        print ("\nQuitting")

