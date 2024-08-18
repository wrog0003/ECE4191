#Written by Warren Rogan
import RPi.GPIO as GPIO
import time
from math import pi, atan2, sqrt

from Motor_code.encoderClassTest import SimpleEncoder
from Sys4_Vision import Sys4_Vision
from ECE4191enums import DIRECTION



# MOTOR OUTPUT PINS (DRIVING)

# Motor Left 
motor1a = 17
motor1b = 27

# Motor Right 
motor2a = 23
motor2b = 24

# MOTOR INPUT PINS (ENCODERS)

# Define the GPIO Pins to recieve the encoder pulses. 

# Motor Left 
motor1cha = 13
motor1chb = 19

# Motor Right 
motor2cha = 5
motor2chb = 6

# Useful data units: meters
wheelDiameter = 0.054 # diameter of the wheel
wheelBase = 0.22 # distance between the centre of both wheels 
wheelBaseCircumference = pi*wheelBase # circumference of the wheel 
distancePerPulse = wheelDiameter*pi/(75*48) # how far the robot can move per pulse of the encoders

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)

# Set up the output pins 
GPIO.setup(motor1a, GPIO.OUT) 
GPIO.setup(motor1b, GPIO.OUT)
GPIO.setup(motor2a, GPIO.OUT)
GPIO.setup(motor2b, GPIO.OUT)

# Set up the PWM pins
pwm1a = GPIO.PWM(motor1a,1000)
pwm1b = GPIO.PWM(motor1b,1000)
pwm2a = GPIO.PWM(motor2a,1000)
pwm2b = GPIO.PWM(motor2b,1000)

def fowards(duty_cycle:float)->list[GPIO.PWM,GPIO.PWM]:
   
    # input: duty cycle between 0 - 100
    # output: return the active pins

    # drive the motor forwards 
    pwm1a.start(0)
    pwm1b.start(duty_cycle)
    pwm2a.start(0)
    pwm2b.start(duty_cycle)

    return [pwm1b,pwm2b] 

def turn(duty_cycle:float,clockWise:bool)->list[GPIO.PWM,GPIO.PWM]:

    # input: duty cycle between 0 - 100, if you want to turn clockwise or anticlockwise 
    # output: return the active pins

    if clockWise:
        pwm1a.start(0)
        pwm1b.start(duty_cycle)
        pwm2a.start(duty_cycle)
        pwm2b.start(0)
        return [pwm1b,pwm2a]
    else:
        pwm1a.start(duty_cycle)
        pwm1b.start(0)
        pwm2a.start(0)
        pwm2b.start(duty_cycle)
        return [pwm1a,pwm2b]

# simple test to make sure that the robot can turn towards the ball 
def turnAtBallTest():
    #init function variables 
    speed = 50
    vision = Sys4_Vision()
    NotAhead = True # init ending variable 
    try :
        while (NotAhead): 
            [direction, temp]= vision.detect() # run vision check 
            if (direction == DIRECTION.Ahead): # if ball is ahead
                NotAhead = False # change to end while loop 
            elif (direction == DIRECTION.CannotFind): # if no ball detected in current frame 
                turn(speed,True)
            elif (direction == DIRECTION.Left):
                turn(speed,False)
            else:
                turn(speed,True) 
            time.sleep(0.2) # delay 200ms 
        # exit and release pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

# simple test to get the robot to find the ball, turn to the ball and get close to the ball 
def hitBallTest():
    # init function variables 
    speed = 50
    vision = Sys4_Vision()
    noHit = True # define stop condition 
    try :
        while (noHit): # while not close enough to ball 
            [direction, temp]= vision.detect() # run vision check 

            if (direction == DIRECTION.Ahead): # if ball ahead
                distance = vision.distanceCalc() # get the distance to ball 
                if (distance <0.01): # if close to ball (1cm)
                    noHit = False # end 
                else: 
                    fowards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind):
                turn(speed,True)
            elif (direction == DIRECTION.Left):
                turn(speed,False)
            else:
                turn(speed,True)
            time.sleep(0.2)
            
        # exit and release pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()


turnAtBallTest() 