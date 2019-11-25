"""Cat chasing code for the Sphero RVR

This is the Rapberry Pi code for a system that will be mounted on
the Sphero RVR. The purpose is to recognize "cat" using the camera
and TensorFlow, then drive the RVR towards the cat until it can't find the cat
anymore. This is clearly a good use of computing resources.

I may add a laser pointer to the pan/tilt mast eventually, because cats
and lasers....
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

import pantilthat       # Pan/Tilt mast controller
from picamera import PiCamera
import tensorflow as tf
import tensorflow_hub as hub

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import RawMotorModesEnum

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


# Package/application modules


######################
# Globals
######################

ALERT_RANGE = 10            # Range at which RVR should stop/turn
SLOW_SPD = 20               # Slow speed (all the speeds need tested)
NORMAL_SPD = 70             # Normal speed
HIGH_SPD = 150              # High speed (max is 255)


######################
# Pre-Main Setup
######################

##### RVR init #####

sys.path.append(os.path.abspath("/home/pi/sphero-sdk"))

loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

##### AI init #####

# module = hub.Module("https://tfhub.dev/inaturalist/vision/embedder/inaturalist_V2/1")
# height, width = hub.get_expected_image_size(module)
# images = ...  # A batch of images with shape [batch_size, height, width, 3].
# features = module(images)  # Features with shape [batch_size, num_features].

##### Blinka/CircuitPython init #####

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)
ads.gain = 1


######################
# Classes and Methods
######################

class RVRpersistence():
    """RVRpersistence - maintain state information about
    the RVR, such as current heading.

    Attributes:
        curHeading - current heading, in absolute degrees
                     from the start heading

    Methods:
        getHeading - returns the current heading
        setHeading - sets the heading attribute to
                     the new heading relative to start
        invertHeading - gives the 180 degree reverse
            heading
    """
    def __init__():
        curHeading = 0

    def getHeading(self):
        return self.curHeading

    def setHeading(self, new_heading):
        self.curHeading = new_heading

    def invertHeading(self):
        if self.curHeading == 180:
            return 0
        if self.curHeading > 180:
            return (self.curHeading - 180)
        return (self.curHeading + 180)


######################
# Functions
######################

##### RVR Control Section #####

async def motor_stall_handler(response):
    """motor_stall_handler is triggered if the drive motor(s)
    stall because of immobiliation.

    Logic here is to:
        1. Stop the rover.
        2. Print a status.
        3. Invert the heading.
        4. Back up for one second.
        5. Go back to looking for the cat.
    """
    stop_rover()
    print('Motor stall response: ', response)
    # NOTE: do NOT reset the heading (RVRdata.setHeading()) because
    # the RVR should end up pointing the same way. If the rover
    # turns around, comment out the following line.
    new_heading = RVRdata.invertHeading
    drive_reverse(SLOW_SPD, new_heading, 1)
    

def set_lights_blue():
    """
    """
    await rvr.led_control.set_multiple_leds_with_enums(
        leds=[
            RvrLedGroups.headlight_left,
            RvrLedGroups.headlight_right
        ],
        colors=[
            Colors.Blue,
            Colors.Blue
        ]
    }


def set_lights_yellow():
    """
    """
    await rvr.led_control.set_multiple_leds_with_enums(
        leds=[
            RvrLedGroups.headlight_left,
            RvrLedGroups.headlight_right
        ],
        colors=[
            Colors.Yellow,
            Colors.Yellow
        ]
    }


def set_lights_green():
    """
    """
    await rvr.led_control.set_multiple_leds_with_enums(
        leds=[
            RvrLedGroups.headlight_left,
            RvrLedGroups.headlight_right
        ],
        colors=[
            Colors.Green,
            Colors.Green
        ]
    }


def flash_green():
    """
    """
    set_lights_green()
    await asyncio.sleep(0.5)
    set_lights_blue()
    await asyncio.sleep(0.5)
    set_lights_green()
    await asyncio.sleep(0.5)
    set_lights_blue()


async def stop_rover():
    """
    """
    await rvr.rw_motors(
        left_mode=RawMotorModesEnum.forward.value,
        left_speed=0,
        right_mode=RawMotorModesEnum.forward.value,
        right_speed=0
    }


async def drive_reverse(input_speed, input_heading, input_time):
    """
    """
    await rvr.drive_control.reset_heading()
    await rvr.drive_control.drive_backward_seconds(
        speed=input_speed,        # Valid speed values are 0-255
        heading=input_heading,    # Valid heading values are 0-359
        time_to_drive=input_time  # Time to roll forward
    )
    await asyncio.sleep(1)     # Delay to allow RVR to drive


async def drive_forward(input_speed, input_heading, input_time):
    """
    """
    await rvr.drive_control.reset_heading()
    await rvr.drive_control.drive_forward_seconds(
        speed=input_speed,        # Valid speed values are 0-255
        heading=input_heading,    # Valid heading values are 0-359
        time_to_drive=input_time  # Time to roll forward
    )
    await asyncio.sleep(1)     # Delay to allow RVR to drive


async def turn_RVR(inDeg, input_speed):
    """turn_RVR

    Perhaps not needed? Drive Rover has a turn function.
    """
    pass


##### Camera/AI Control Section #####

def is_cat(imageFile):
    """
    """
    # Look for cat
    # If cat, take better picture. 
    pass


def take_picture(outFile):
    """
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
    """
    """
    # Point the camera
    pantilthat.pan(panval)     # positive is left from camera's POV
    pantilthat.tilt(tiltval)   # negative is "up"


def scan_for_cat():
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
    """
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
    # 3.3V yields ~6.4mV/in.
    cur_value, cur_volt = read_adc()
    return(cur_value)


def read_adc():
    # Read the ADC
    curVal = chan.value
    curVolt = chan.voltage
    print("Value: %d  Voltage: %0.3f\n" % (curVal, curVolt))
    return (curVal, curVolt)


##### Main #####

async def main():
    """Abstract main() into a function. Normally exits after execution.

    A function abstracting the main code in the module, which
    allows it to be used for libraries as well as testing (i.e., it can be
    called as a script for testing or imported as a library, without
    modification).
    """

    print("System Startup\n")

    # Class instantiation for persistent RVR data
    RVRdata = RVRpersistence

    print("RVR Startup\n")

    # Get RVR's attention
    await rvr.wake()
    await rvr.enable_motor_stall_notify(is_enabled=True)
    await rvr.on_motor_stall_notify(handler=motor_stall_handler)
    await asyncio.sleep(2)     # Give RVR time to wake up
    await rvr.reset_yaw()
    await set_lights_blue()      # default running color is blue

    print("System and RVR startup complete\n")

    # module_url = "https://tfhub.dev/google/nnlm-en-dim128/2"
    # embed = hub.KerasLayer(module_url)
    # embeddings = embed(["A long sentence.", "single-word",
    #                       "http://example.com"])
    # print(embeddings.shape)  #(3,128)

    print("Operating loop.\n")

    while True:
        # Check battery state
            # If low or critical
                # sleep the rover?
                # Orderly shutdown to save flash integrity

        # Scan for cat with camera
        cat_range, cat_heading = scan_for_cat()
        if cat_range > -1:
            print("Cat detected!\n")
            # Drive towards cat if it's safe
            if not scan_for_hazard:
                print("Clear to drive.\n")
                # Translate camera direction to heading
                # Translate range to drive time
                await drive_forward(NORMAL_SPD, RVRdata.getHeading(), 5)
                await asyncio.sleep(2)
            else:
                stop_rover()
                print("Hazard avoidance triggered.\n")
                new_heading = RVRdata.invertHeading
                drive_reverse(SLOW_SPD, new_heading, 1)
        else:
            # No cat.
            print("No cat.\n")  # Just a statement to make it valid Python
            await asyncio.sleep(15) # Chill out for 15 seconds.

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

