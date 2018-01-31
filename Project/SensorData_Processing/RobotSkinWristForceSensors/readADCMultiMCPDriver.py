         #!/usr/bin/env python

# Written by Limor "Ladyada" Fried for Adafruit Industries, (c) 2015
# This code is released into the public domain

import time
import os
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

SPICLK = 18
SPIMISO = 23
SPIMOSI = 24
SPICSA = 12
SPICSB = 13


clockpin = SPICLK
mosipin = SPIMOSI
misopin = SPIMISO
cspinA = SPICSA
cspinB = SPICSB

# SPICLK, SPIMOSI, SPIMISO, SPICS

DEBUG = 0
DEBUG2 = 1 
# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
#def readadc(adcnum, clockpin, mosipin, misopin, cspin):
def readadcA(adcnum):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspinA, True)

        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspinA, False)     # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(mosipin, True)
                else:
                        GPIO.output(mosipin, False)
                commandout <<= 1
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
                adcout <<= 1
                if (GPIO.input(misopin)):
                        adcout |= 0x1

        GPIO.output(cspinA, True)
        
        adcout >>= 1       # first bit is 'null' so drop it
        return adcout


# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
#def readadc(adcnum, clockpin, mosipin, misopin, cspin):
def readadcB(adcnum):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspinB, True)

        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspinB, False)     # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(mosipin, True)
                else:
                        GPIO.output(mosipin, False)
                commandout <<= 1
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
                adcout <<= 1
                if (GPIO.input(misopin)):
                        adcout |= 0x1

        GPIO.output(cspinB, True)
        
        adcout >>= 1       # first bit is 'null' so drop it
        return adcout


# change these as desired - they're the pins connected from the
# SPI port on the ADC to the Cobbler

# set up the SPI interface pins
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICSA, GPIO.OUT)
GPIO.setup(SPICSB, GPIO.OUT)

#GPIO.setwarnings(False)

# touch switch on board: 
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


