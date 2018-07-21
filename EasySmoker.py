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
#import PID as PID

# Pin Setups
GPIO.setwarnings(False)
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
setTemp = 29 # Test set temperature, C
sum = 0
pTemp = 15
iTemp = 0.4
fanSpeed = 100

class PID:

    # PID controller based on proportional band in standard PID form https://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
    # u = Kp (e(t)+ 1/Ti INT + Td de/dt)
    # PB = Proportional Band
    # Ti = Goal of eliminating in Ti seconds
    # Td = Predicts error value at Td in seconds

    def __init__(self, PB, Ti, Td):
        self.CalculateGains(PB, Ti, Td)

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.u = 0.0

        self.Derv = 0.0
        self.Inter = 0.0
        self.Inter_max = abs(0.5 / self.Ki)

        self.Last = 150

        self.setTarget(0.0)

    def CalculateGains(self, PB, Ti, Td):
        self.Kp = -1 / PB
        self.Ki = self.Kp / Ti
        self.Kd = self.Kp * Td

    def update(self, Current):
        # P term
        error = Current - self.setPoint
        self.P = self.Kp * error + 0.5

        # I term
        dT = time.time() - self.LastUpdate
        self.Inter += error * dT
        self.Inter = max(self.Inter, -self.Inter_max)
        self.Inter = min(self.Inter, self.Inter_max)

        self.I = self.Ki * self.Inter

        # D term
        self.Derv = (Current - self.Last) / dT
        self.D = self.Kd * self.Derv

        # PID
        self.u = self.P + self.I + self.D

        # Update for next cycle
        self.error = error
        self.Last = Current
        self.LastUpdate = time.time()

        return self.u

    def setTarget(self, setPoint):
        self.setPoint = setPoint
        self.error = 0.0
        self.Inter = 0.0
        self.Derv = 0.0
        self.LastUpdate = time.time()

    def setGains(self, PB, Ti, Td):
        self.CalculateGains(PB, Ti, Kd)
        self.Inter_max = abs(0.5 / self.Ki)

    def getK(self):
        return self.Kp, self.Ki, self.Kd

def tempMeasure():
    # Get Temp from pointed tip PT100 probe
    pointyTemp = max31865.max31865(csPin0, misoPin, mosiPin, clkPin).readTemp()

    # Get Temp from blunt tip PT100 probe
    bluntTemp = max31865.max31865(csPin1, misoPin, mosiPin, clkPin).readTemp()

    return pointyTemp, bluntTemp

def displayTemp(pTemp, bTemp):
    # Start LCD Code
    mylcd = I2C_LCD_driver.lcd()

    if pTemp is not None and bTemp is not None:
        print('Pointy temperature is: {}'.format(round(pTemp, 1)))
        print('Blunt temperature is: {}'.format(round(bTemp, 1)))
    else:
        print('Failed to get reading. Try again!')

    # Display Temp & Humidity vals on LCD screen
    mylcd.lcd_display_string("Pointy: %0.1fC" % pointyTemp,1)
    mylcd.lcd_display_string("Blunt: %0.1fC" % bluntTemp,2)

def fanSpeed(currentTemp, desiredTemp):
    global fanSpeed, sum
    tempMeasure()
    displayTemp()
    diff = currentTemp - desiredTemp
    sum = sum + diff
    pDiff = diff * pTemp
    iDiff = sum * iTemp
    fanSpeed = pDiff + iDiff
    if fanSpeed > 100:
        fanspeed = 100
    elif fanSpeed < 15:
        fanspeed = 0
    if sum > 100:
        sum = 100
    elif sum < -100:
        sum = -100
    myPWM.ChangeDutyCycle(fanspeed)
    return()

try:
    GPIO.setwarnings(False)
    myPWM = GPIO.PWM(LEDPin, 100)
    myPWM.start(50)
    while True:
        tempVals = tempMeasure()
        pointyTemp = tempVals[0]
        bluntTemp = tempVals[1]
        print(pointyTemp, bluntTemp)
        displayTemp(pointyTemp, bluntTemp)
        fanSpeed(pointyTemp, setTemp)
        time.sleep(2)
except KeyboardInterrupt:
    GPIO.cleanup()