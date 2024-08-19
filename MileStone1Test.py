#Written by Warren Rogan
import RPi.GPIO as GPIO
import time
from math import pi, atan2, sqrt

#from Motor_code.encoderClassTest import SimpleEncoder
from Sys4_Vision import Sys4_Vision
from ECE4191enums import DIRECTION
from Motor_code.encoderClass import SimpleEncoder


ANTICLOCKWISE = False
CLOCKWISE = True

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
    print("entered")
    speed = 20
    vision = Sys4_Vision()
    NotAhead = True # init ending variable 
    try :
        while (NotAhead): 
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction)
            if (direction == DIRECTION.Ahead): # if ball is ahead
                NotAhead = False # change to end while loop 
                pwm1a.stop()
                pwm1b.stop()
                pwm2a.stop()
                pwm2b.stop()
            elif (direction == DIRECTION.CannotFind): # if no ball detected in current frame 
                turn(speed, ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                turn(speed, ANTICLOCKWISE)
            else:
                turn(speed,CLOCKWISE) 
            time.sleep(0.2) # delay 200ms 
        # exit and release pins 
        
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

# simple test to get the robot to find the ball, turn to the ball and get close to the ball 
def hitBallTestComplex():
    # init function variables 
    speed = 50
    pauseTime = 0.2
    vision = Sys4_Vision()
    noHit = True # define stop condition 
    try :
        while (noHit): # while not close enough to ball 
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction.name)
            print(distance)
            if (direction == DIRECTION.Ahead): # if ball ahead
                speed = 100
                pauseTime = 0.5
                if (distance <0.3):
                    speed = 20
                    vision.tolerence = 150
                    fowards(speed)
                if (distance <0.2): # if close to ball 
                    fowards(30)
                    time.sleep(3)
                    noHit = False # end 
                    pwm1a.stop()
                    pwm1b.stop()
                    pwm2a.stop()
                    pwm2b.stop()
                else: 
                    fowards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind):
                speed = 20
                pauseTime = 0.3
                turn(speed,ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                speed = 20
                pauseTime =0.15
                turn(speed,ANTICLOCKWISE)

            else:
                turn(speed,CLOCKWISE)
                speed = 20
                pauseTime =0.15
            time.sleep(pauseTime)
            
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

# simple ball hitter that turns towards ball then goes until it hits ball
def hitBallTestSimple():
    # init function variables 
    speed = 15
    pauseTime = 0.1
    vision = Sys4_Vision()
    vision.tolerence = 15 # decrease tollerence 
    notAhead = True # define turing stop condition 
    notHit = True # define running stop condition 
    distance = 100 # predefine distance 
    try :
        while notAhead:
            (direction, temp, distance)= vision.detect() # run vision check 
            if (direction == DIRECTION.Ahead): # if directly ahead
                notAhead = False
                pwm1a.stop()
                pwm1b.stop()
                pwm2a.stop()
                pwm2b.stop()
                speed = 100 
            elif(direction==DIRECTION.CannotFind):
                speed = 50
                turn(speed,ANTICLOCKWISE)
            elif(direction == DIRECTION.Left):
                speed = 15
                turn(speed,ANTICLOCKWISE)
            else: # right 
                speed =15 
                turn(speed,CLOCKWISE)
            time.sleep(pauseTime)
        
        #move forward
        pauseTime = distance/0.25
        fowards(100)
        time.sleep(pauseTime)
        #should hit ball

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


hitBallTestSimple()