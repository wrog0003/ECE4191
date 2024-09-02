from gpiozero import Button, RotaryEncoder

# Set up a simple encoder class to track miltiple encoders on single system
class SimpleEncoder:
    # define initialisation parameters to set up all sections 
    def __init__(self,Apin:int,Bpin:int) -> None:
        self.Apin = Button(Apin, pull_up=True) # high = logic 1, low = logic 0
        self.Bpin = Button(Bpin, pull_up=True) 
        self.encoderCount = 0 # initialise encoder count to 0
        self.encoderOldCount = 0 
    
        self.clockWise = False # facing in from outside
        #self.DirectionList = [self.clockWise]*3
        # set up interrupts (rising and falling)
        self.Apin.when_pressed = self.encoderCallA 
        #self.Bpin.when_pressed = self.encoderCallB

        #self.Apin.when_released = self.encoderCall
        #self.Bpin.when_released = self.encoderCall

    # interrupt callback functions
    def encoderCallA(self,channel):
        self.encoderCount+=1 # increment encoder count
        # if (self.Bpin.value):
        #     self.clockWise = True
        # else:
        #     self.clockWise = False

    def encoderCallB(self,channel):
        self.encoderCount+=1 # increment encoder count
        # if (self.Apin.value):
        #     self.clockWise = False
        # else:
        #     self.clockWise = True 

    # def encoderCall(self,channel):
    #     self.encoderCount+=1

    #get the state of the encoder, current count, direction, old count
    def getValues(self)->tuple[int,bool,int]:
        temp = self.encoderOldCount
        self.encoderOldCount = self.encoderCount
        return [self.encoderCount,self.clockWise,temp]
    
    #function call to relase pins
    def end(self)->None:
        self.Apin.close()
        self.Bpin.close() 

class BetterEncoder(RotaryEncoder):
    def __init__(self,Apin:int,Bpin:int)->None:
        RotaryEncoder.__init__(self,Apin,Bpin) # call the init of the parent class 
        self.encoderOldCount = 0 # create a save of the old encoder
    
    def getValues(self)->tuple[int,bool,int]:
        temp = self.encoderOldCount
        self.encoderOldCount = self.steps
        return [self.steps,True,temp]
    def end(self)->None:
        self.close()
     
# drive definitions 
#   L   R       direction
#   CCW CW      forward
#   CW  CCW     backward
#   CCW CCW     right
#   CW  CW      left

# Emma's attempt at using the rotary encoder class 

class Encoder:

    # initalise encoder 
    def _init_ (self,Apin:int,Bpin:int) -> None:
        self.encoderCount = 0 
        self.encoderOldCount = 0 
        RotaryEncoder(self, Apin, Bpin) # inialise the rotatry encoder from gpio zero 
        
