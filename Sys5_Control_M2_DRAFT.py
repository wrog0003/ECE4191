# overall brains script for robot, written by Emma Vladicic and Warren Rogan 

from ECE4191enums import STATE, DIRECTION, ACTION
from Motor_code.encoderClass import SimpleEncoder
from Sys4_Vision import Sys4_Vision

from time import sleep
from math import pi, atan2, sqrt, sin, cos

import RPi.GPIO as GPIO
import GLOBALSM1 # for physical values of the robot i.e. wheel diameter, distance b/w wheeels etc   

# Define important variables 

# Define clockwise and anti-clockwise as Boolean variables 
ANTICLOCKWISE = False
CLOCKWISE = True

# Define physical GPIO pins on the Rpi 
# Motor Left (to drive motors)
motor1a = 17
motor1b = 27

# Motor Right (to drive motors)
motor2a = 23
motor2b = 24

# Encoder Left (to recieve data from encoders)
motor1cha = 13
motor1chb = 19

# Encoder Right (to recieve data from encoders)
motor2cha = 5
motor2chb = 6

# Wheel bias (used to callibrate differences in wheel movement)
duty_cycle_bias = 0.963


#Milestone 1 control top level class 

class Sys5_Control:

    # initialisation of the class
    def __init__(self) -> None:

        GPIO.setmode(GPIO.BCM) # set pin types 

        # Set up the motor output pins 
        GPIO.setup(motor1a, GPIO.OUT) 
        GPIO.setup(motor1b, GPIO.OUT)
        GPIO.setup(motor2a, GPIO.OUT)
        GPIO.setup(motor2b, GPIO.OUT)

        # Set up the PWM pins
        self.pwm1a = GPIO.PWM(motor1a,1000)
        self.pwm1b = GPIO.PWM(motor1b,1000)
        self.pwm2a = GPIO.PWM(motor2a,1000)
        self.pwm2b = GPIO.PWM(motor2b,1000)
        self._stop() # prevent random movements
        
        # initalise the vision system
        self.vision = Sys4_Vision()

        #Position control

        # define the initial position of the robot as (0,0) with a rotation of 0 
        self.x_pos =0
        self.y_pos = 0
        self.rot =0 

        self.State = None # State is the movement of the robot i.e. Left, Right, Forward, Backward
        # initalise to none bc robot is not moving

        self.EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
        self.EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor

            
    def _forwards(self,duty_cycle:float)->ACTION:
    
        # duty cycle = controls speed of robot (0 - 100)
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1a.start(0)
        self.pwm1b.start(duty_cycle)
        self.pwm2a.start(0)
        self.pwm2b.start(max(duty_cycle*duty_cycle_bias,5))

        return ACTION.FORWARD

    def _backwards(self, duty_cycle:float)->ACTION:
    
        # duty cycle = controls speed of robot (0 - 100)
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1b.start(0)
        self.pwm1a.start(duty_cycle)
        self.pwm2b.start(0)
        self.pwm2a.start(max(duty_cycle,5))

        return ACTION.BACKWARD 

    def _turn(self,duty_cycle:float,clockWise:bool)->ACTION:

        # input: 
        # duty cycle = controls speed of robot (0 - 100)
        # bool = dirction to turn, clockwise or anticlockwise 
        # output: return the state of the robot

        if clockWise: # turn clockwise 
            self.pwm1a.start(0)
            self.pwm1b.start(duty_cycle)
            self.pwm2a.start(max(duty_cycle*duty_cycle_bias,5))
            self.pwm2b.start(0)
            return ACTION.RIGHT
        else: # turn anti-clockwise
            self.pwm1a.start(duty_cycle)
            self.pwm1b.start(0)
            self.pwm2a.start(0)
            self.pwm2b.start(max(duty_cycle*duty_cycle_bias,5))
            return ACTION.LEFT

    def _stop(self)->None: # stop movement of robot 
        self.pwm1a.stop()
        self.pwm1b.stop()
        self.pwm2a.stop()
        self.pwm2b.stop()

    # Manual exit function to prevent loss of pin control
    def _exemptExit(self)->None:
        self._stop()
        GPIO.cleanup()
        self.EncoderL.end()
        self.EncoderR.end()
        self.vision.disconnect() 
    
    # Release all Pins
    def release(self)->None:
        self._stop()
        self.vision.disconnect()
        GPIO.cleanup()
        sleep(0.1) # ensure that every peripheral is released 

    # Position tracking 
    def _updatePos(self, x_old:float, y_old:float, rot_old:float)->list[float]:

        # inputs 
        # self = direction that the robot is moving 
        # x_old, y_old, rot_old = previous x, y coordinate and rotation of robot

        # output 
        # x, y, rot = current x, y  coordinates and rotation of robot

        # get the number of pulses the encoder has counted
        [newL, dirL, oldL] = self.EncoderL.getValues()
        [newR, dirR, oldR] = self.EncoderR.getValues()

        # calculate the difference in the number of encoder pulses in the time between the last call
        delL = abs(newL-oldL)
        delR = abs(newR-oldR)

        # calculate the average travelled distance 
        distanceAvg = ((delL*GLOBALSM1.distancePerPulse)+(delR*GLOBALSM1.distancePerPulse))/2 
    
        # reset coordinates and rotation to 0 
        x = 0
        y = 0 
        rot = 0 

        # calculate the change in x, y and angle positions
        dx = distanceAvg*cos(rot*pi/180)
        dy = distanceAvg*sin(rot*pi/180)
        dtheta = distanceAvg*360/GLOBALSM1.wheelBaseCircumference

        # determine direction that the robot have travelled in the previous timestep & update coordinates
        if self.State == ACTION.FORWARD: 
            rot = rot_old
            y = y_old + dy 
            x = x_old + dx

        elif self.State ==ACTION.BACKWARD:
            rot = rot_old
            y = y_old - dy
            x = x_old - dx

        elif self.State == ACTION.LEFT:
            x = x_old
            y = y_old
            rot = rot_old + dtheta

        elif self.State == ACTION.RIGHT:
            x= x_old
            y = y_old
            rot = rot_old - dtheta

        # limit the angle to the correct domain
        if rot > 180:
            rot -=360 
        elif rot <-180:
            rot +=360 

        return x,y,rot