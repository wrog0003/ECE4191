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
    
    def run(self)->None:
        while True:
            #get sensor data
            [desiredDirection, boundry] = self.vision.detect()
            ballCount = self.ballCounter.getCount()







            
