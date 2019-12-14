# RVR-Cat-Chaser

## Overview

The RVR Cat Chaser uses a number of components on a Sphero RVR to identify a cat and then chase the cat.
This may be extened in the future to include a cat treat dispenser and a cat-paw-friendly button to teach the cat
to play games with the RVR. My daughter suggested I create a BB8-like language of beeps and other tones
to train the cat to understand the RVR. That's a bit down the road, though.

## Hardware Dependencies

- Sphero RVR
- Raspberry Pi 3 B+
- Raspberry Pi Camera v2
- Adafruit Proto-Pi Hat
- Adafruit tall Raspberri Pi headers
- Pimoroni pan/tilt hat
- Adafruit pan/tilt mast
- Maxbotix LV-EZ0 ultrasonic rangefinder
- Adafruit ADS1115 16-bit ADC
- Adafruit PowerBoost 1000C
- 2700 mAh liPo battery with JST-PH connector
- 90 degree headers
- Assorted dupont female-to-female wires
- Longer Pi Camera cable
- USB-A male to USB-A male pigtai; six inches will do
- Micro-USB male to Micro-USB male cable; six to eight inches will do
- M2.5 nylon sstandoff kit
- Mounting plate for camera, rangefinder, and ADC (design to be created and added)
- (Optional) Extra RVR battery

## Software Dependencies

- Raspbian Buster, patched
- Sphero RVR Python SDK
- Python 3.7 or higher environment
- Python3 RPi.GPIO library
- Adafruit Blinka library
- Adafruit ADS1x15 CircuitPython library
- Pimaroni pan/tilt library
- AdafruitIO account (io.adafruit.com) - bridge between RVR and IFTTT
- IFTTT account (If This Then That) - for notifications to your phone

## Setup and Usage

Horribly scruffy, but, in general:

- sudo apt update
- sudo apt -y dist-upgrade
- sudo raspi-config and turn off console output to /dev/ttyS0 !!!
- solder tall Pi headers, and 90 degree headers on the Proto-Pi Hat (layout diagram to be added)
- attach the pan/tilt mast to the pan/tilt hat
- attach the camera, MaxBotix, and ADS1115 to the mounting plate (I prototyped with cardboard but recommend 1/8" laser-cut acrylic)
- attach the mounting plate to the front of the pan/tilt mast using hardware that comes with the pan/tilt mast
- mount the Raspberry Pi on the RVR cover plate using M2.5 nylon standoffs and existing holes
- connect the servo cables to the pan/tilt hat (identification of pan and tilt servos to be added)
- connect RX, TX, and GND from the Proto-Pi Hat to the serial header on the RVR (remember RX-TX and TX-RX)
- connect 3.3v, GND, SCL, and SDA to from the Proto-Pi Hat to the ADS1115
- connect 3.3v and GND from the Proto-Pi Hat to the Vdd and GND on the MaxBotix
- connect 3.3v from the Proto-Pi Hat to BW on the MaxBotix (unclear if this is needed, given the data sheet)
- connect AN on the MaxBotix to A0 on the ADS1115
- connect the USB-A port on the RVR to the USB-A port on the PowerBoost 1000C
- connect the Micro-USB port on the PowerBoost to the Raspberry Pi Micro-USB power port
- connect the liPo tot he PowerBoost 1000C
- NOTE: the PowerBoost board will get wickedly hot, so it will help to have it hanging out in the air for cooling
- mount the Proto-Pi Hat on the Raspberry Pi
- mount the pan/tilt Hat on the Proto-Pi Hat
- Add entries for your AdafruitIO Username and Key in ~/secrets.py (it's a Python dict, see "secrets.py"), then add the directory with the secrets.py file to your PYTHONPATH env variable
- Configure a feed on your AdafruitIO account named "RVR_cat_chase"
- Configure an action on IFTTT to send you a notification if data shows up on the AdafruitIO feed - this is so the RVR can tell you when the battery is too low to operate, and everything is shutting itself down cleanly
- "clone this repo, power everything on, run software" (this is obviously glossing over a LOT of detail, which will need to be fleshed out)

