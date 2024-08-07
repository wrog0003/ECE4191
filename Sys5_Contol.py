from ECE4191enums import STATE, DIRECTION
from Sys1_Balls import Sys1_Balls
from Sys3_SLAM import Sys3_SLAM
from Sys4_Vision import Sys4_Vision

import RPi.GPIO as GPIO


class Sys5_Control:
    # init function to set up everything
    def __init__(self,motorPins:list[int],encoderPins:list[int],switchPins:list[int], sensorPin:int) -> None:
        # init variables
        self.State = STATE.null

        #clear pins
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        #instatinate classes
        self.vision = Sys4_Vision()
        self.localisation = Sys3_SLAM(encoderPins)
        self.ballCounter = Sys1_Balls(sensorPin)

        # connect pins
        self.motorLPins = motorPins[0:1]
        self.motorRPins = motorPins[2:3]
        GPIO.setup(self.motorLPins[0],GPIO.OUT)
        GPIO.setup(self.motorLPins[1],GPIO.OUT)
        GPIO.setup(self.motorRPins[0],GPIO.OUT)
        GPIO.setup(self.motorRPins[1],GPIO.OUT)

    def _unloadBall(self)->None:
        pass 
        # TODO
    
    def _return2Home(self)->None:
        pass 
        #TODO
    
    def _turnAround(self,amount: float )->None:
        pass 
        #TODO

    
    def _move2Ball(self)->None:
        #TODO
        pass 
    
    #brains of the robot 
    def run(self)->None:
        while True:
            #get sensor data
            [desiredDirection, boundry] = self.vision.detect()
            full = self.ballCounter.isFull() 
            [X, Y, rot, home] = self.localisation.getLocationRot() 

            #decide what to do 
            if home:
                self._unloadBall()
                #TODO
            elif full:
                self._return2Home()
                #TODO
            elif boundry:
                self._turnAround(180)
            elif desiredDirection== DIRECTION.CannotFind:
                self._turnAround(45)
            elif desiredDirection == DIRECTION.Ahead:
                self._move2Ball()
            elif desiredDirection == DIRECTION.Left:
                self._turnAround(5)
            elif desiredDirection == DIRECTION.Right:
                self._turnAround(-5)









            
