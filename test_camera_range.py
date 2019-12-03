"""Test camera and rangefinder routines for cat chasing RVR

Tests camera panning, image taking, and rangefinding.
and lasers....
"""
##############################################################################
# Author:      Phil Moyer (phil@moyer.ai)
# Date:        December 2019
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

import smbus
import os
import sys
from time import sleep
import io
import time
from datetime import datetime, timedelta

# Third-party modules

import pantilthat       # Pan/Tilt mast controller
from picamera import PiCamera

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

##### Camera/AI Control Section #####

def is_cat(imageFile):
    """is_cat() uses TensorFlow to determine if the camera sees a cat.
    
    Arguements: none
    
    Returns:
    	True if cat is detected
    	Fals if cat is NOT detected
    """
    # Look for cat
    # If cat, take better picture. 
    pass


def take_picture(outFile):
    """take_picture() captures a single high-resolution image
    from the Raspberry Pi camera.
    
    Note: this image will need to be downgraded to 299x299 and converted to
    black and white for TensorFlow.
    
    Arguements:
    	outFile - defines where to store the captured image
    	
    Returns: nothing
    """
    # Capture an image
    with PiCamera() as camera:
        camera.vflip = True
        camera.hflip = True
        # camera.iso = 400
        camera.contrast = 15
        camera.sharpness = 35
        camera.saturation = 20
        # sleep(2)
        # camera.shutter_speed = camera.exposure_speed
        camera.shutter_speed = 0   # auto
        # camera.exposure_mode = 'off'
        camera.capture(outFile, format="png")


def take_picture_stream():
    """Take pictures continuously and stream to memory

    This may be a more useful way to capture pictures
    to look for cat, but it will depend on how
    TensorFlow works.

    Note that the RVR will not be controlled while this is
    running unless we use asynchronous threads.
    """
    with picamera.PiCamera() as camera:
        camera.vflip = True
        camera.hflip = True
        camera.contrast = 15
        camera.sharpness = 35
        camera.saturation = 20
        camera.shutter_speed = 0   # auto
        stream = io.BytesIO()
        for foo in camera.capture_continuous(stream, format='jpeg'):
            # Truncate the stream to the current position (in case
            # prior iterations output a longer image)
            stream.truncate()
            stream.seek(0)
            if process(stream):
                break


def point_camera(panval, tiltval):
    """point_camera() uses the pan/tilt mast to point the camera in
    a particular azimuth and elevation.
    
    Note: the mapping from the arguement values to actual direction and
    elevation angle has not been determined. We will need to do this
    experimentally
    
    Arguements:
    	pan - direction to pan the camera. Positive is left.
    	tilt - angle to tilt the camera. Negative is up.
    	
    Returns: nothing
    """
    # Point the camera
    pantilthat.pan(panval)     # positive is left from camera's POV
    pantilthat.tilt(tiltval)   # negative is "up"


def scan_for_cat():
	"""scan_for_cat() is the framework for scanning the surroundings to
	look for cat. It uses is_cat() and point_camera() to move the camera from
	side to side looking for cat.
	
	Arguements: none
	Returns:
		If cat detected:
		  range - distance to cat in inches
		  heading - the direction in which cat was located
		If cat not detected:
			-1,-1
	"""
    picture_file = "current_view.png"
    #
    # Point the camera in a given direction
    # Remember, pan left is positive, and tilt "up" is negative
    for azimuth in range(20, -20, -1):
    	take_picture(picture_file)
    	# Cat there?
    	if is_cat(picture_file):
    		print("Cat!\n")
    		# get range in inches to target
    		cat_range = adc_to_range()
    		# transform azimuth into heading
    		# return range, heading
    return(-1,-1)


def scan_for_hazard():
    """scan_for_hazard() is called just before starting off to chase the cat.
    It uses point_camera() to tilt from zero to "up" in order to find any
    overhead obstacles that could damage the instruments on the pan/tilt mast.
    
    Arguements: none
    
    Returns:
    	True - hazard detected
    	False - no hazard detected, free to move
    """
    pass


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
    cur_value, cur_volt = read_adc()

    # This formula was extracted from several hundred observations
    # collected with measured distance.

    # y = 0.03110x - 7.35300
 
    cur_range = (0.03110 * cur_value) - 7.35300
    
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
    print("Value: %d  Voltage: %0.3f\n" % (curVal, curVolt))
    return (curVal, curVolt)

def shutdown_pi():
	"""shutdown_pi() executes a clean shutdown to avoid damage to flash memory.
	
	Arguements: none
	
	Returns: nothing
	"""
	command = "/usr/bin/sudo /sbin/shutdown -h now"
	import subprocess.process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
	output = process.communicate()[0]
	# NOTREACHED
	return

##### Main #####

def main():
    """Abstract main() into a function. Normally exits after execution.

    A function abstracting the main code in the module, which
    allows it to be used for libraries as well as testing (i.e., it can be
    called as a script for testing or imported as a library, without
    modification).
    """

    print("System Startup\n")

    # module_url = "https://tfhub.dev/google/nnlm-en-dim128/2"
    # embed = hub.KerasLayer(module_url)
    # embeddings = embed(["A long sentence.", "single-word",
    #                       "http://example.com"])
    # print(embeddings.shape)  #(3,128)

    print("Operating loop.\n")
    
    PicCount = 1  # Keep track of picture count

    for PanAngle in range(-45, 45):
        PicFile = "cat%0.3d.jpg" % (PicCount)
        take_picture()        
        cRange = adc_to_range()
        print("Picture %s, range $0.3f, azimuth %d" % (PicFile, cRange, PanAngle))
        PicCount += 1


######################
# Main
######################

# The main code call allows this module to be imported as a library or
# called as a standalone program because __name__ will not be properly
# set unless called as a program.

if __name__ == '__main__':
    try:
        loop.run_until_complete(
            main()
        )

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            rvr.close()
        )

    finally:
        if loop.is_running():
            loop.close()
            


