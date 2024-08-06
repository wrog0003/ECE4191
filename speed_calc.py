# ECE4191 6/08/2024 
# Using Raspberry Pi to recieve encoder data and calculate the speed of the motors. 

import RPi.GPIO as GPIO 
from time import sleep

# Setup the GPIO Pins to recieve the encoder pulses. 
GPIO.cleanup() # Removes any predefined allocation of GPIO Pins/Avoids conflicts.
motor1cha = 5
motor1chb = 6
motor2cha = 16
motor2chb = 26

GPIO.setmode(GPIO.BCM)

GPIO.setup(motor1cha, GPIO.IN) 
GPIO.setup(motor1chb, GPIO.IN)
GPIO.setup(motor2cha, GPIO.IN)
GPIO.setup(motor2chb, GPIO.IN)


# Capture edges of the encoder pulses 

# Determine the direction of motion by determining the leading signal. 
# This will be different depending on if it is the left or the right motor. 


# Calculate the speed (rad/sec) by calculating the time between the rising edges. 
