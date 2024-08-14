# overall brains script for robot, written by Warren Rogan 

from ECE4191enums import STATE, DIRECTION
from Sys1_Balls import Sys1_Balls
from Sys3_SLAM import Sys3_SLAM
from Sys4_Vision import Sys4_Vision

from time import sleep

import RPi.GPIO as GPIO

'''
functions are either timed or untimed
timed functions get called then do their task including a delay then return to main loop
untimed functions get called then return to main loop
'''



class Sys5_Control:
    # init function to set up everything
    angleRotationSpeed = 5 #time in seconds per degree of rotation 
    rotTollerence = 2 # degrees tollerence for rotation
    locTollerence = 0.05 # tollerence for location in meters 

    def __init__(self,motorPins:list[int],encoderPins:list[int],switchPins:list[int], sensorPin:int) -> None:
        # init variables
        self.State = STATE.null
        self.M1Complete = False #if it has completed hitting the ball 

        #clear pins
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        #instantiate classes
        self.vision = Sys4_Vision()
        self.localization = Sys3_SLAM(encoderPins,switchPins[0])

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
    def _return2Home(self)->None:
        angle = 0 #TODO work out angle that the robot should turn 2 reltive to start
        if (abs(self.rot-angle)>Sys5_Control.tollerence):
            #check which direction to turn
            # if diff > 180 degrees turn against conventinal direction
            #else turn in convential direction 
            if (abs(self.rot-angle)>180): #against convention
                self._turn(self.rot >angle)
            else: 
                self._turn(not (self.rot>angle)) 
        else:
            self._stop()
            #check location 
            #if location tollerence is bad then move forward
        
        


    #turn around by an amount, + angle is anticlockwise
    # timed 
    def _turnAround(self,amount: float )->None:
        pauseTime = abs(amount*Sys5_Control.angleRotationSpeed)
        #rotate in correct direction
        if (amount >0):
            self._turn(False)
        else:
            self._turn(True)
        sleep(pauseTime) # wait until it should have turned to the correct angle 


    
    #brains of the robot 
    def run(self)->None:
        try:
            while True:
                #get sensor data
                [desiredDirection, boundry] = self.vision.detect()
                
                [X, Y, self.rot, home] = self.localization.getLocationRot()  #make sure self.rot is limited to -180->180

                #decide what to do 
                if desiredDirection== DIRECTION.CannotFind:
                    self._turnAround(45)
                elif desiredDirection == DIRECTION.Ahead:
                    self._forward()
                elif desiredDirection == DIRECTION.Left:
                    self._turnAround(5)
                elif desiredDirection == DIRECTION.Right:
                    self._turnAround(-5)
        except KeyboardInterrupt:
            # Cleanup GPIO settings before exiting
            GPIO.cleanup()
            print("GPIO cleanup done. Exiting gracefully.")








            
