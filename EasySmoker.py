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

GPIO.setup(nextPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(selectPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(LEDPin, GPIO.OUT)

def LEDon():
    GPIO.output(LEDPin, True)

def LEDoff():
    GPIO.output(LEDPin, False)

# Base Parameters
TempInterval = 3 #Frequency to record temperatures, seconds
PIDCycleTime = 5 #Frequency to update control loop, seconds
u_min = 0.15 # Maintenance Level
u_max = 1.0
Parameters = {'mode': 'Off', 'target':225, 'PB': 60.0,\
              'Ti': 180.0, 'Td': 45.0, 'CycleTime': 20, 'u': 0.15,\
              'PMode': 2.0, 'program': False,\
              'ProgramToggle': time.time()}  #60,180,45 held +- 5F

#Start controller
Control = PID.PID(Parameters['PB'],Parameters['Ti'],Parameters['Td'])
Control.setTarget(Parameters['target'])



while True:
    # Get Temp from pointed tip PT100 probe
    pointyTemp = max31865.max31865(csPin0, misoPin, mosiPin, clkPin).readTemp()

    # Get Temp from blunt tip PT100 probe
    bluntTemp = max31865.max31865(csPin1, misoPin, mosiPin, clkPin).readTemp()

    print('Pointy temperature is: {}'.format(round(pointyTemp,1)))
    print('Blunt temperature is: {}'.format(round(bluntTemp,1)))

    # Start LCD Code
    mylcd = I2C_LCD_driver.lcd()

    if pointyTemp is not None and bluntTemp is not None:
        print('Pointy Temp={0:0.1f}C Blunt Temp = {1:0.1f}%'.format(pointyTemp, bluntTemp))
        #print(round(temperature,2))
        #print(round(humidity,2))
    else:
        print('Failed to get reading. Try again!')

    # Display Temp & Humidity vals on LCD screen
    mylcd.lcd_display_string("Pointy: %0.1fC" % pointyTemp,1)
    mylcd.lcd_display_string("Blunt: %0.1fC" % bluntTemp,2)




    time.sleep(TempInterval)
