# code to detect a line using an RGB sensor

import RPi.GPIO as GPIO
import time 
import adafruit_tcs34725
import board
import busio

# set up I2C communication with the sensor
i2c = busio.I2C(board.SCL, board.SDA)

# create a colour sensor oject
sensor = adafruit_tcs34725.TCS34725(i2c)

# enable the sensor's internal LED for better colour reading 
sensor.enable_led = True

def read_colour():

    # get the RGB values that the sensor is reading 

    r, g, b, c = sensor.color_raw 

    # clear refers to the measurement of total light intensity falling on the sensor

    print(f"Raw Colour Data - Red:{r}, Green:{g}, Blue: {b}, Clear:{c}")


if __name__ == "__main__":
    from time import sleep 
    try:
        while(True):
            read_colour()
                    
            sleep(0.1) #reduce load
    except KeyboardInterrupt:
        GPIO.cleanup() 
    

