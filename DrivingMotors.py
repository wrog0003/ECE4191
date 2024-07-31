# ECE4191 29/07/2024 Emma Vladicic 
# Basic progam to test functionality of motors with Raspberry Pi 

# LOOK INTO USING Robot functionality 
# https://projects.raspberrypi.org/en/projects/physical-computing/14


import RPi.GPIO as GPIO 
import time 

# configure the GPIO pins MIGHT NEED TO CHANGE 

# inputs for motor 1 
IN1 = 17 # input 1
IN2 = 18 # input 2 
EN = 19 # enable 

# enable GPIO 

GPIO.setmode(GPIO.BCM)  
GPIO.setup(IN1, GPIO.OUT) 
GPIO.setup(IN2, GPIO.OUT) 
GPIO.setup(EN, GPIO.OUT) 

# PWM ?? 


# set enable pin high 

def motor_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)






