#This class if used to know the location of the robot at any time 
# it should feed into the Control class and instantiate 2 encoder classes 
#Define X as the forward direction based on the robot start position 
#Define Y as right releteive to the robot start point 

#interrupts based on https://roboticsbackend.com/raspberry-pi-gpio-interrupts-tutorial/
import sys 
import signal
from math import pi
import RPi.GPIO as GPIO

class Encoder:
    def __init__(self,pinA: int, pinB: int)->None:
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        self.ACount = 0
        self.BCount = 0
        self.pinA = pinA
        self.pinB = pinB
        GPIO.setup(self.pinA,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pinB,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
        self.direction = False


    # set to detect interrupts 
    def run(self)->None:
        GPIO.add_event_detect(self.pinA,GPIO.RISING,callback=Adetected)
        GPIO.add_event_detect(self.pinB,GPIO.RISING,callback=Bdetected)


    def Adetected(self,channel)->None:
        self.ACount = self.ACount +1
        if (GPIO.input(self.pinB)):
            self.direction = True

        # read pin states 
    def Bdetected(self,channel)->None:
        #read pin states 
        self.BCount = self.Bcount+1
        if (GPIO.input(self.pinA)):
            self.direction = False 
    # return values to localiser to get working values 
    def getValues(self)->tuple[int,bool]: 
        return [self.ACount+self.BCount,self.direction]
    
    def signal_handler(sig, frame):
        GPIO.cleanup() # used to clean up GPIO, may be needed to prevent blocking the GPIO access 
        sys.exit(0)


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, 
            callback=button_pressed_callback, bouncetime=100)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

        

class Localisation:
    #radius should be in mm 
    def __init__(self,radius:float,pinA1:int,pinB1:int,pinA2:int,pinB2:int) -> None:
        #radius should be in mm 
        self.X = 0
        self.Y = 0
        self.Rot = 0 
        self.motorLEncoder = Encoder(pinA1,pinB1)
        self.motorREncoder = Encoder(pinA2,pinB2)
        self.oldLpulses = 0
        self.oldRpulses = 0
        self.circumference = radius*2*pi

    #updates localisation, should be called at a high frequency to ensure that the localisation is correct 
    def updatePos(self)->None:
        [newPulseL,directionL] = self.motorLEncoder.getValues()
        [newPulseR,directionR] = self.motorREncoder.getValues() 
        delLpulses = newPulseL-self.oldLpulses
        delRpulses = newPulseR-self.oldRpulses
        # check if forwards or backwards
        if directionL != directionR:
            # check if forward 
        if directionL == directionR:
            # check which direction rotation 

    #data that the control system cares about 
    def getPos(self)-> list[float]:
        return [self.X,self.Y]
    
    def getRot(self)->float:
        return self.Rot 
