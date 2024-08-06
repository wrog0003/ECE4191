import RPi.GPIO as GPIO


class Sys1_Balls:
    #setup 
    def __init__(self, sensorPin:int)->None:
        self.sensor = sensorPin
        self.count = 0 
        GPIO.setup(self.sensor,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

    #depoit all balls 
    def empty(self)->None:
        self.count = 0

    # check if full
    def isFull(self)->int:
        return self.count >= 4
    
    #
    def run(self)->None: 

    def _addBall(self)->None:
        self.count =self.count+1
