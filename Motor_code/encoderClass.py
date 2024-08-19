from gpiozero import Button 

# Set up a simple encoder class to track miltiple encoders on single system
class SimpleEncoder:
    # define initialisation parameters to set up all sections 
    def __init__(self,Apin:int,Bpin:int) -> None:
        self.Apin = Button(Apin, pull_up=True) # high = logic 1, low = logic 0
        self.Bpin = Button(Bpin, pull_up=True) 
        self.encoderCount = 0 # initialise encoder count to 0
        self.encoderOldCount = 0 
        self.clockWise = False # facing in from outside
        # set up interrupts (rising and falling)
        self.Apin.when_pressed = self.encoderCallA 
        self.Bpin.when_pressed = self.encoderCallB
        self.Apin.when_released = self.encoderCall
        self.Bpin.when_released = self.encoderCall

    # interrupt callback functions
    def encoderCallA(self,channel):
        self.encoderCount+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = True

    def encoderCallB(self,channel):
        self.encoderCount+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = False

    def encoderCall(self,channel):
        self.encoderCount+=1

    #get the state of the encoder, current count, direction, old count
    def getValues(self)->tuple[int,bool,int]:
        temp = self.encoderOldCount
        self.encoderOldCount = self.encoderCount
        return [self.encoderCount,self.clockWise,temp]
    
    #function call to relase pins
    def end(self)->None:
        self.Apin.close()
        self.Bpin.close() 


# drive definitions 
#   L   R       direction
#   CCW CW      forward
#   CW  CCW     backward
#   CCW CCW     right
#   CW  CW      left