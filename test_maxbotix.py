"""test_maxbotix tests the ADC and Maxbotix rangefinder configuration.

This is just a simple program to exercise the maxbotix rangefinder.
"""
##############################################################################
# Author:      Phil Moyer (phil@moyer.ai)
# Date:        November 2019
#
# License: This program is released under the MIT license. Any
# redistribution must include this header.
##############################################################################

# Remember the style guide: http://www.python.org/dev/peps/pep-0008/
#
# Functions, variables, and attributes should be lowercase_underscore
# Protected instance attributes should be _leading_underscore
# Private instance attributes should be __double_leading_underscore
# Classes and exceptions should be CapitalizedWord
# Module-level constants should be ALL_CAPS

######################
# Import Libraries
######################

# Standard libraries modules

import numpy as np
import smbus
import os
import sys
from time import sleep
import io
import time
from datetime import datetime, timedelta

# Third-party modules

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


# Package/application modules


######################
# Globals
######################



######################
# Pre-Main Setup
######################

##### Blinka/CircuitPython init #####

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)
ads.gain = 1


######################
# Classes and Methods
######################


######################
# Functions
######################

##### Rangefinder Section #####

# NOTE: this uses the MaxBotix LV-EZ0 ultrasonic rangefinder.
# Pin 3 -> ADC
# Pin 6 -> 3.3v
# Pin 7 -> GND

# It also uses an ADS1115 16-bit I2C ADC with programmable gain.
# VCC -> 3.3v


def adc_to_range():
    """adc_to_range provides range in inches from the
    rangefinder (MaxBotix LV-EZ)

    Returns:
        range in inches
    """
    # 3.3V yields ~6.4mV/in.
    cur_value, cur_volt = read_adc()
    cur_range = cur_volt * 6.4 # to give us inches?
    print("Value %d, Voltage %0.6f, range (in) %0.3f" % (cur_value, cur_volt, cur_range))
    return(cur_range)


def read_adc():
    """
    read_adc() simply reads the values from the analog-to-digital
    converter and returns them. The ADS1115 returns both a "value"
    and the voltage. In our case, voltage will be most useful.
	
    Arguements: none
	
    Returns:
        Value
        Voltage (need to check units)
    """
    # Read the ADC
    curVal = chan.value
    curVolt = chan.voltage
    # print("Value: %d  Voltage: %0.3f" % (curVal, curVolt))
    return (curVal, curVolt)


##### Main #####

def main():
    """Abstract main() into a function. Normally exits after execution.

    A function abstracting the main code in the module, which
    allows it to be used for libraries as well as testing (i.e., it can be
    called as a script for testing or imported as a library, without
    modification).
    """

    while True:
    	adc_to_range()
    	sleep(5)

    # NOTREACHED

    # Exit from async "run until complete"
    return


######################
# Main
######################

# The main code call allows this module to be imported as a library or
# called as a standalone program because __name__ will not be properly
# set unless called as a program.

if __name__ == '__main__':
    main()
            

