# RPi Smoker Controller #
# FHT #
# V1.0 #
# 2018-06-19 #


# Add folders to system path to
# include other downloaded modules
# Change system path as required if modules
# are in a different folder than Home
# May need to add __init__.py file to folder
import sys
sys.path.insert(0,'/home/pi/')

# Import Other Modules
import time
from I2C_LCD_driver import I2C_LCD_driver # For 16x2 LCD Display
from MAX31865 import max31865 # Allows for connecting RPi to PTDs
import RPi.GPIO as GPIO
import RPi.PWM as PWM
import PID as PID

# Pin Setups
GPIO.setmode(GPIO.BCM)
csPin0 = 8 #CE0 Pin
csPin1 = 7 #CE1 Pin
misoPin = 9
mosiPin = 10
clkPin = 11
nextPin = 26
selectPin = 13
LEDPin = 19

#GPIO.setup(nextPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
#GPIO.setup(selectPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(LEDPin, GPIO.OUT)

"""
def LEDon():
    GPIO.output(LEDPin, True)
def LEDoff():
    GPIO.output(LEDPin, False)
"""

# Base Parameters
TempInterval = 3 #Frequency to record temperatures, seconds
Kp = 0.75
Ki = 0.01
Kd = 0.001
Channel = 1
setTemp = 28.5 # Test set temperature, C
min_duty_cycle = 0
max_duty_cycle = 1

while True:
    # Get Temp from pointed tip PT100 probe
    pointyTemp = max31865.max31865(csPin0, misoPin, mosiPin, clkPin).readTemp()

    # Get Temp from blunt tip PT100 probe
    bluntTemp = max31865.max31865(csPin1, misoPin, mosiPin, clkPin).readTemp()

    # Start LCD Code
    mylcd = I2C_LCD_driver.lcd()

    if pointyTemp is not None and bluntTemp is not None:
        print('Pointy temperature is: {}'.format(round(pointyTemp, 1)))
        print('Blunt temperature is: {}'.format(round(bluntTemp, 1)))
    else:
        print('Failed to get reading. Try again!')

    # Display Temp & Humidity vals on LCD screen
    mylcd.lcd_display_string("Pointy: %0.1fC" % pointyTemp,1)
    mylcd.lcd_display_string("Blunt: %0.1fC" % bluntTemp,2)

    # PID Control








    time.sleep(TempInterval)
