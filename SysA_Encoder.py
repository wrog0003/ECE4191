from gpiozero import Button

# Set up a simple encoder class to track miltiple encoders on single system
class SysA_Encoder:
    '''
    A simple encoder class that connects to two pins and counts the number of rising edges on pin A
    '''

    def __init__(self,Apin:int) -> None:
        '''
        This function creates an instance of the encoder class and sets up the pins
        
        Inputs:
            self: instance of the class
            Apin: the GPIO pin number to connected to the encoder
            Bpin: the second GPIO pin connected to the encoder, no longer in use
        '''
        self.Apin = Button(Apin, pull_up=True) # high = logic 1, low = logic 0 
        self.encoderCount = 0 # initialise encoder count to 0 
        '''The number of encoder pulses current'''
        self.encoderOldCount = 0 
        '''The number of encoder pulses the last time the encoder was checked'''
    
        self.clockWise = False # facing in from outside
        self.Apin.when_pressed = self.encoderCallA 

    # interrupt callback functions
    def encoderCallA(self,channel):
        '''
        Callback function to increment the encoder count'''
        self.encoderCount+=1 # increment encoder count



    #get the state of the encoder, current count, direction, old count
    def getValues(self)->tuple[int,bool,int]:
        '''
        This function is used to pass distance information to the robots controller and update the old count
        
        Outputs:
            the current encoder count
            the direction (legacy)
            the old encoder count'''
        temp = self.encoderOldCount
        self.encoderOldCount = self.encoderCount
        return [self.encoderCount,self.clockWise,temp]
    
    def __str__(self) -> str:
        return f'Encoder with count {self.encoderCount}'
    
    #function call to relase pins
    def __del__(self)->None:
        '''
        This function releases control of the button instances to prevent lockup of the GPIO pins to safely destroy the class instance'''
        self.Apin.close()
