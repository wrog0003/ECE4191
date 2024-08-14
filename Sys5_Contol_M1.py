# overall brains script for robot, written by Warren Rogan 

from ECE4191enums import STATE, DIRECTION
from Sys1_Balls import Sys1_Balls
from Sys3_SLAM import Sys3_SLAM
from Sys4_Vision import Sys4_Vision

from time import sleep
from math import sqrt, atan2, pi

import RPi.GPIO as GPIO

'''
functions are either timed or untimed
timed functions get called then do their task including a delay then return to main loop
untimed functions get called then return to main loop
'''



class Sys5_Control:
    # init function to set up everything
    angleRotationSpeed = 5 #time in seconds per degree of rotation 
    rotTolerance = 2 # degrees tollerence for rotation
    locTolerance = 0.05 # tollerence for location in meters 

    def __init__(self,motorPins:list[int],encoderPins:list[int],switchPins:list[int], sensorPin:int) -> None:
        # init variables
        self.State = STATE.null
        self.M1Complete = False #if it has completed hitting the ball 
        self.X =0
        self.Y =0
        self.rot=0
        self.hitBall = False # check if it hit the ball

        #clear pins
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        #instantiate classes
        self.ball = Sys1_Balls(switchPins[0])
        self.localization = Sys3_SLAM(encoderPins,switchPins[0])
        self.vision = Sys4_Vision()

        # connect pins
        self.motorLPins = motorPins[0:1]
        self.motorRPins = motorPins[2:3]
        GPIO.setup(self.motorLPins[0],GPIO.OUT)
        GPIO.setup(self.motorLPins[1],GPIO.OUT)
        GPIO.setup(self.motorRPins[0],GPIO.OUT)
        GPIO.setup(self.motorRPins[1],GPIO.OUT)

    #set the mode to get the robot moving forward, untimed
    def _forward(self)->None:
        GPIO.output(self.motorLPins[1],GPIO.LOW)
        GPIO.output(self.motorRPins[1],GPIO.LOW)
        GPIO.output(self.motorLPins[0],GPIO.HIGH)
        GPIO.output(self.motorRPins[0],GPIO.HIGH)

    #spin in a direction, untimed
    def _turn(self,ClockWise:bool)->None:
        GPIO.output(self.motorLPins[1],GPIO.LOW)
        GPIO.output(self.motorRPins[1],GPIO.LOW)
        GPIO.output(self.motorLPins[0],GPIO.LOW)
        GPIO.output(self.motorRPins[0],GPIO.LOW)
        if (ClockWise):
            GPIO.output(self.motorLPins[0],GPIO.HIGH)
            GPIO.output(self.motorRPins[1],GPIO.HIGH)
        else: #anticlockwise 
            GPIO.output(self.motorRPins[0],GPIO.HIGH)
            GPIO.output(self.motorLPins[1],GPIO.HIGH)

    #stop, untimed 
    def _stop(self)->None:
        PIO.output(self.motorLPins[1],GPIO.LOW)
        GPIO.output(self.motorRPins[1],GPIO.LOW)
        GPIO.output(self.motorLPins[0],GPIO.LOW)
        GPIO.output(self.motorRPins[0],GPIO.LOW)
    
    # tries to return to home, untimed
    def _return2Home(self)->bool:
        angle = atan2(self.Y,self.X)*180/pi # get the angle towards home absolute
    
        if (abs(self.rot-angle)>Sys5_Control.rotTolerance): # if the angle is greater than the tolerance 
            if (abs(self.rot-angle)>180): #against convention as the arc in conventional method would mean going around a further way
                self._turn(self.rot >angle) #against the direction that convention expects
            else: # is conventinal way as it is less than 180 degrees
                self._turn(not (self.rot>angle)) # based on the angle that needs to be achieved 
            return False 
        else:
            self._stop() # prevent over rotation 
            if (sqrt(self.X^2+self.Y^2)>Sys5_Control.locTolerance):
                self._forward() # move toward home 
                return False
            else:
                return True # is at home
        
    # #turn around by an amount, + angle is anticlockwise
    # # timed 
    # def _turnAround(self,amount: float )->None:
    #     pauseTime = abs(amount*Sys5_Control.angleRotationSpeed)
    #     #rotate in correct direction
    #     if (amount >0):
    #         self._turn(False)
    #     else:
    #         self._turn(True)
    #     sleep(pauseTime) # wait until it should have turned to the correct angle 


    #brains of the robot 
    def run(self)->None:
        try:
            while True:
                if (self.State == STATE.atHome):
                    GPIO.cleanup()
                    break; # finish up M1 
                #get sensor data
                [desiredDirection, boundry] = self.vision.detect()
                
                [self.X, self.Y, self.rot, home] = self.localization.getLocationRot()  #make sure self.rot is limited to -180->180
                if(not self.hitBall): #before it has hit the ball
                    #decide what to do 
                    if desiredDirection == DIRECTION.CannotFind:
                        self._turn(False)
                    elif desiredDirection == DIRECTION.Ahead:
                        self._forward()
                    elif desiredDirection == DIRECTION.Left:
                        self._turn(False)
                    elif desiredDirection == DIRECTION.Right:
                        self._turn(True)
                    else: #once it has hit the ball
                        home =self._return2Home(); #return to home
                        if (home): #if it has reached home 
                            self.State = STATE.atHome

                sleep(0.1) # wait before next check

        except KeyboardInterrupt:
            # Cleanup GPIO settings before exiting
            GPIO.cleanup()
            print("GPIO cleanup done. Exiting gracefully.")








            
