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
import statistics
import subprocess

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
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from Adafruit_IO import Client, Feed, Data, RequestError


# Package/application modules


######################
# Globals
######################

ALERT_RANGE = 10            # Range at which RVR should stop/turn
SLOW_SPD = 20               # Slow speed (all the speeds need tested)
NORMAL_SPD = 70             # Normal speed
HIGH_SPD = 150              # High speed (max is 255)
TF_WIDTH = 299              # Picture widtth for TensorFlow
TF_HEIGHT = 299             # Picture height for TensorFlow
TF_BW = True                # Whether TensorFlow wants B&W (unused ATM)
MOVE_RVR = False            # Whether the RVR should move or not (for testing)


######################
# Pre-Main Setup
######################

##### Adafruit IO init #####

# Get AdafruitIO details from the secrets.py file
try:
    from secrets import secrets
except ImportError:
    print("AdafruitIO secrets are kept in secrets.py, please add them there!")
    raise

aio_username= secrets['aio_username']
aio_key = secrets['aio_key']
aio = Client(aio_username, aio_key)

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
    def __init__(self):
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
    if MOVE_RVR:
        drive_reverse(SLOW_SPD, new_heading, 1)
    

async def set_lights_blue():
    """set_lights_blue() turns the RVR lights blue.
    
    Arguements: none
    Returns: nothing
    """

    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 0, 255]]
    )


async def set_lights_yellow():
    """set_lights_yellow() turns the RVR headlights yellow.
    
    Arguements: none
    Returns: nothing
    """

    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [255, 255, 0]]
    )


async def set_lights_green():
    """set_lights_green() turns the RVR headlights green.
    
    Arguements: none
    Returns: nothing
    """

    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
    )


async def set_lights_red():
    """set_lights_red() turns the RVR headlights red.
    
    Arguements: none
    Returns: nothing
    """

    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [255, 0, 0]]
    )


async def flash_green():
    """flash_green() flashes the RVR headlights green, then blue, then green,
    then back to blue. This is intended to be used as a visual signal that
    the cat has been detected.
    
    Arguements: none
    Returns: nothing
    """
    await set_lights_green()
    await asyncio.sleep(0.5)
    await set_lights_blue()
    await asyncio.sleep(0.5)
    await set_lights_green()
    await asyncio.sleep(0.5)
    await set_lights_blue()


async def stop_rover():
    """stop_rover() uses the RVR raw motor interface to completely stop
    the rover. This can be used as a normal stop, if needed, or to freeze
    the rover in case some hazard is present, such as a low table or other
    obstacle that could damage the instruments on the pan/tilt mast.
    
    Arguments: none
    Returns: nothing
    """
    if not MOVE_RVR:
        return
    await rvr.rw_motors(
        left_mode=RawMotorModesEnum.forward.value,
        left_speed=0,
        right_mode=RawMotorModesEnum.forward.value,
        right_speed=0
    )


async def drive_reverse(input_speed, input_heading, input_time):
    """drive_reverse() drives the RVR on a reverse heading at
    the given speed and for the given time.
    
    Arguements:
        speed (0 to 255)
        heading (0 to 359)
        time (seconds)
        
    Returns: nothing
    """
    if not MOVE_RVR:
        return
    await rvr.drive_control.reset_heading()
    await rvr.drive_control.drive_backward_seconds(
        speed=input_speed,        # Valid speed values are 0-255
        heading=input_heading,    # Valid heading values are 0-359
        time_to_drive=input_time  # Time to roll forward
    )
    await asyncio.sleep(1)     # Delay to allow RVR to drive


async def drive_forward(input_speed, input_heading, input_time):
    """drive_forward() drives the RVR forward at the given speed,
    on the given heading, and for the given time.
    
    Arguements:
        speed (0 to 255)
        heading (0 to 359)
        time (seconds)
        
    Returns: nothing
    """
    if not MOVE_RVR:
        return
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
    """is_cat() uses TensorFlow to determine if the camera sees a cat.
    
    Arguements: none
    
    Returns:
        True if cat is detected
        Fals if cat is NOT detected
    """
    # Look for cat
    # If cat, take better picture. 
    return False


def take_picture(outFile):
    """take_picture() captures a single black and white frame
    in the dimensions (pixels) required by the TensorFlow
    training set.

    Arguements:
        outFile - defines where to store the captured image

    Returns: nothing
    """
    # Capture an image
    with PiCamera() as camera:
        camera.vflip = True
        camera.hflip = True
        camera.contrast = 15
        camera.sharpness = 35
        camera.saturation = 20
        camera.shutter_speed = 0   # auto
        camera.color_effects = (128,128)     # sets the camera to black and white
        # camera.PiResolution(width=TF_WIDTH, height=TF_HEIGHT)
        camera.resolution = (TF_WIDTH, TF_HEIGHT)
        camera.capture(outFile, format="jpeg")


def take_picture_hd(outFile):
    """take_picture() captures a single high-resolution image
    from the Raspberry Pi camera.
    
    Note: this image will need to be downgraded to 299x299 and converted to
    black and white for TensorFlow.
    
    Arguements:
        filepath - defines where to store the captured image
        
    Returns: nothing
    """
    # Capture an image
    with PiCamera() as camera:
        camera.vflip = True
        camera.hflip = True
        camera.contrast = 15
        camera.sharpness = 35
        camera.saturation = 20
        camera.shutter_speed = 0   # auto
        camera.capture(outFile, format="png")


##### Perhaps delete this since probably not needed #####
def convert_pic_to_tf(inFile, outFile, outWidth, outHeight, black_and_white=True):
    """convert_pic_to_tf process pic to TensorFlow requirements.

    Arguements:
        inFile - name and path of input file
        outFile - name and path of output file
        outWidth - width of output image (pixels)
        outHeight - height of output image (pixels)
        black_and_white - boolean indicating change image to B&W
    """
    pass


##### Perhaps delete this since probably not needed #####
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


def scan_for_cat(loc_pic_num):
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

    pic_count = loc_pic_num

    # Point the camera in a given direction
    # Remember, pan left is positive, and tilt "up" is negative

    for PanAngle in range(0, -100, -10):
        pantilthat.pan(PanAngle)
        picture_file = "cat%0.3d.jpg" % (pic_count)
        pic_count += 1
        sleep(1)
        take_picture(picture_file)
        # Cat there?
        if is_cat(picture_file):
            print("Cat!\n")
            flash_lights_green()
            # Get a good picture of the cat for verification
            hd_pic_file = "actual_cat%0.3d.jpg" % (pic_count)
            take_picture_hd(hd_pic_file)
            # get range in inches to target
            cat_range = adc_to_range()
            # PanAngle is heading? ### Check Sphero SDK for heading specification. ###
            cat_heading = PanAngle
            # return(cat_range, cat_heading)

    for PanAngle in range(0, 100, 10):
        pantilthat.pan(PanAngle)
        picture_file = "cat%0.3d.jpg" % (pic_count)
        pic_count += 1
        sleep(1)
        take_picture(picture_file)
        # Cat there?
        if is_cat(picture_file):
            print("Cat!\n")
            flash_lights_green()
            # Get a good picture of the cat for verification
            hd_pic_file = "actual_cat%0.3d.jpg" % (pic_count)
            take_picture_hd(hd_pic_file)
            # get range in inches to target
            cat_range = adc_to_range()
            # PanAngle is heading? ### Check Sphero SDK for heading specification. ###
            cat_heading = PanAngle
            # return(cat_range, cat_heading)

    return(-1,-1)


def scan_for_hazard():
    """scan_for_hazard() is called just before starting off to chase the cat.
    It uses point_camera() to tilt from 80 to "up" in order to find any
    overhead obstacles that could damage the instruments on the pan/tilt mast.
    
    Arguements: none
    
    Returns:
        True - hazard detected
        False - no hazard detected, free to move
    """
    for cam_tilt in range(80,40,5):
        cur_range = adc_to_range()
        if 10 > cur_range:
            print("Hazard detected!")
            center_camera()
            return True
    center_camera()
    return False


##### Rangefinder Section #####

# NOTE: this uses the MaxBotix LV-EZ0 ultrasonic rangefinder. The
# MaxBotix series have different ranges and "sight" patterns, so
# check the data sheets for the one you want to use. EZ0 is the
# narrowest and longest range, so that's why I use it.

# Pin 3 -> ADC
# Pin 6 -> 3.3v
# Pin 7 -> GND

# It also uses an ADS1115 16-bit I2C ADC with programmable gain.
# VCC -> 3.3v

def adc_to_range():
    """adc_to_range provides range in inches from the
    rangefinder (MaxBotix LV-EZ0)

    Returns:
        range in inches
    """
    cur_value, cur_volt = read_adc()

    # This formula was extracted from several hundred observations
    # collected with measured distance. Fortunately, the value:range
    # relationship is linear.

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
    return (curVal, curVolt)


def center_camera():
    """center_camera() surprisingly centers the camera.

    Arguements: none
    Returns: nothing
    """

    # Interesting. The pan/tilt mast is looking down too far at 90,
    # and the pan is not centered at zero.
   
    # Fix it with device-specific values.

    pantilthat.tilt(80)
    pantilthat.pan(-12)


def shutdown_pi():
    """shutdown_pi() executes a clean shutdown to avoid damage to flash memory.
    
    Arguements: none
    
    Returns: nothing
    """
    try:
        rvr_alert = aio.feeds('rvr-cat-chase')
        aio.send_data(rvr_alert.key, 4)
    except RequestError:
        print("Cannot establish connection to RVR_cat_chase.")

    subprocess.run(["/usr/bin/sudo","/sbin/shutdonw","-h","now"])

    # NOTREACHED
    return


##### Main #####

async def main():
    """Abstract main() into a function. Normally exits after execution.

    A function abstracting the main code in the module, which
    allows it to be used for libraries as well as testing (i.e., it can be
    called as a script for testing or imported as a library, without
    modification).
    """

    print("System Startup\n")
    center_camera()
    pic_count = 1
    await set_lights_red()
    sleep(2)
    await set_lights_yellow()
    sleep(2)
    await set_lights_green()

    # Class instantiation for persistent RVR data
    RVRdata = RVRpersistence()

    print("RVR Startup\n")

    # Get RVR's attention
    await rvr.wake()
    await rvr.enable_motor_stall_notify(is_enabled=True)
    await rvr.on_motor_stall_notify(handler=motor_stall_handler)
    await asyncio.sleep(2)     # Give RVR time to wake up
    await rvr.reset_yaw()
    await set_lights_blue()      # default running color is blue
    RVRdata.setHeading(0)

    print("System and RVR startup complete\n")

    # module_url = "https://tfhub.dev/google/nnlm-en-dim128/2"
    # embed = hub.KerasLayer(module_url)
    # embeddings = embed(["A long sentence.", "single-word",
    #                       "http://example.com"])
    # print(embeddings.shape)  #(3,128)

    print("Operating loop.\n")

    while True:
        # Check battery state
        battery_percentage = await rvr.get_battery_percentage()
        print('Battery percentage: ', battery_percentage)
        if 5 >= battery_percentage["percentage"]:
            # If low or critical
            print("##### Critically low RVR battery. Shutting down. #####")
            rvr.sleep()
            shutdown_pi()  # Orderly shutdown to save flash integrity

        # Scan for cat with camera
        cat_range, cat_heading = scan_for_cat(1)

        # pic_count += 1
        center_camera()
        if cat_range > -1:
            print("Cat detected!\n")
            # Drive towards cat if it's safe
            if not scan_for_hazard:
                print("Clear to drive.\n")
                # Translate camera direction to heading
                RVRdata.setHeading(cat_heading)
                # Translate range to drive time
                if MOVE_RVR:
                    await drive_forward(NORMAL_SPD, RVRdata.getHeading(), 5)
                    await asyncio.sleep(2)
            else:
                await stop_rover()
                print("Hazard avoidance triggered.\n")
                if MOVE_RVR:
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
            
