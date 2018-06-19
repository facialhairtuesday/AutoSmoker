# RPi Smoker Controller #
# FHT #
# V1.0 #
# 2018-06-19 #


# Add home folder to system path
# to include other downloaded modules
# Change system path as required if modules
# are in a different folder than Home
# May need to add __init__.py file to folder
import sys
sys.path.insert(0,'/home/pi/')

# Import Modules
import Adafruit_DHT
import I2C_LCD_driver as driver
from time import *
import MAX31865

# Start LCD Code
mylcd = driver.lcd()

# Setup DHT22 Sensor
sensor = Adafruit_DHT.DHT22
pin = 4

# Get Temp & Humidity values (C & %)
humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

if humidity is not None and temperature is not None:
    print('Temp={0:0.1f}C Humidity = {1:0.1f}%'.format(temperature, humidity))
    print(round(temperature,2))
    print(round(humidity,2))
else:
    print('Failed to get reading. Try again!')

# Display Temp & Humidity vals on LCD screen
mylcd.lcd_display_string("Temp: %dC" % temperature,1)
mylcd.lcd_display_string("Humidity: %d%%" % humidity,2)
