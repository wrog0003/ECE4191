import RPi.GPIO as GPIO

# this class should be connected to a normally closed switch so that if the connection is bad it is easy to detect 

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
    def isFull(self)->bool:
        return self.count >= 4
    
    #start detecting balls
    def run(self)->None: 
        GPIO.add_event_detect(self.sensor,GPIO.FALLING,callback=_addBall,bouncetime=100)

    def _addBall(self,channel)->None:
        self.count =self.count+1
