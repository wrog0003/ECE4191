from gpiozero import Button 
import RPi.GPIO as GPIO
import time

#simple encoder class to track miltiple encoders on single system
class SimpleEncoder:
    #init to set up all sections 
    def __init__(self,Apin:int,Bpin:int) -> None:
        self.Apin = Button(Apin, pull_up=True) 
        self.Bpin = Button(Bpin, pull_up=True) 
        self.encoderCount = 0
        self.clockWise = False #facing in from outside
        self.Apin.when_pressed = self.encoderCallA
        self.Bpin.when_pressed = self.encoderCallB
        self.Apin.when_released = self.encoderCall
        self.Bpin.when_released = self.encoderCall

    # interrupt callback functions
    def encoderCallA(self,channel):
        encoder_count+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = False

    def encoderCallB(self,channel):
        encoder_count+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = True

    def encoderCall(self,channel):
        encoder_count+=1

    #get the state of the encoder 
    def getValues(self)->tuple[int,bool]:
        return [self.encoderCount,self.clockWise]
    
    #function call to relase pins
    def end(self)->None:
        self.Apin.close()
        self.Bpin.close() 


def directionTest