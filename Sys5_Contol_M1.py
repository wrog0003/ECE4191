# overall brains script for robot, written by Warren Rogan 

from ECE4191enums import STATE, DIRECTION, ACTION
from Sys3_SLAM import Sys3_SLAM
from Sys4_Vision import Sys4_Vision

from time import sleep
from math import sqrt, atan2, pi

import RPi.GPIO as GPIO
import GLOBALSM1 #for physical values 

#DEFINE
ANTICLOCKWISE = False
CLOCKWISE = True

#PINS
# Motor Left 
motor1a = 17
motor1b = 27

# Motor Right 
motor2a = 23
motor2b = 24

# Encoder Left 
motor1cha = 13
motor1chb = 19

# Encoder Right 
motor2cha = 5
motor2chb = 6

#Wheel bias
duty_cycle_bias = 0.95


#Milestone 1 controll top level class 
class Sys5_Control:

    def __init__(self) -> None:

        GPIO.setmode(GPIO.BCM) # set pin types 
        # Set up the output pins 
        GPIO.setup(motor1a, GPIO.OUT) 
        GPIO.setup(motor1b, GPIO.OUT)
        GPIO.setup(motor2a, GPIO.OUT)
        GPIO.setup(motor2b, GPIO.OUT)

        # Set up the PWM pins
        self.pwm1a = GPIO.PWM(motor1a,1000)
        self.pwm1b = GPIO.PWM(motor1b,1000)
        self.pwm2a = GPIO.PWM(motor2a,1000)
        self.pwm2b = GPIO.PWM(motor2b,1000)

        # create class instances 
        self.vision = Sys4_Vision()
        self.location = Sys3_SLAM() 
    
    def forwards(self,duty_cycle:float)->ACTION:
    
        # input: duty cycle between 0 - 100
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1a.start(0)
        self.pwm1b.start(duty_cycle)
        self.pwm2a.start(0)
        self.pwm2b.start(max(duty_cycle*duty_cycle_bias,5))

        return ACTION.FORWARD

    def backwards(self,duty_cycle:float)->ACTION:
    
        # input: duty cycle between 0 - 100
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1b.start(0)
        self.pwm1a.start(duty_cycle)
        self.pwm2b.start(0)
        self.pwm2a.start(max(duty_cycle*duty_cycle_bias,5))

        return ACTION.BACKWARD 

    def turn(self,duty_cycle:float,clockWise:bool)->ACTION:

        # input: duty cycle between 0 - 100, if you want to turn clockwise or anticlockwise 
        # output: return the state of the robot

        if clockWise:
            self.pwm1a.start(0)
            self.pwm1b.start(duty_cycle)
            self.pwm2a.start(max(duty_cycle*duty_cycle_bias,5))
            self.pwm2b.start(0)
            return ACTION.RIGHT
        else:
            self.pwm1a.start(duty_cycle)
            self.pwm1b.start(0)
            self.pwm2a.start(0)
            self.pwm2b.start(max(duty_cycle*duty_cycle_bias,5))
            return ACTION.LEFT

    def stop(self)->None:
        self.pwm1a.stop()
        self.pwm1b.stop()
        self.pwm2a.stop()
        self.pwm2b.stop()











            
