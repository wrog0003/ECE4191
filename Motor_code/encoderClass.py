from gpiozero import button

# Set up a simple encoder class to track miltiple encoders on single system
class SimpleEncoder:
    # define initialisation parameters to set up all sections 
    def __init__(self,Apin:int,Bpin:int) -> None:
        self.Apin = Button(Apin, pull_up=True) # high = logic 1, low = logic 0
        self.Bpin = Button(Bpin, pull_up=True) 
        self.encoderCount = 0 # initialise encoder count to 0
        self.clockWise = False # facing in from outside

        # Set up interrupts
        self.Apin.when_pressed = self._encoder_callback
        self.Bpin.when_pressed = self._encoder_callback

    def _encoder_callback(self):
        # Update encoder count based on the direction
        if self.Apin.value == self.Bpin.value:
            self.encoderCount += 1
            self.clockWise = True
        else:
            self.encoderCount -= 1
            self.clockWise = False

    #get the state of the encoder 
    def getValues(self)->tuple[int,bool]:
        return [self.encoderCount,self.clockWise]
    
    #function call to relase pins
    def end(self)->None:
        self.Apin.close()
        self.Bpin.close() 