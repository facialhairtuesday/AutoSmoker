import sys
sys.path.insert(0,'/home/pi/')

import Adafruit_DHT
import I2C_LCD_driver as driver
from time import *

mylcd = driver.lcd()

sensor = Adafruit_DHT.DHT22

pin = 4

humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

if humidity is not None and temperature is not None:
    print('Temp={0:0.1f}C Humidity = {1:0.1f}%'.format(temperature, humidity))
    print(round(temperature,2))
    print(round(humidity,2))
else:
    print('Failed to get reading. Try again!')

mylcd.lcd_display_string("Temp: %dC" % temperature,1)
mylcd.lcd_display_string("Humidity: %d%%" % humidity,2)
