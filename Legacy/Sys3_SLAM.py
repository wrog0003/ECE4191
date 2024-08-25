import RPi.GPIO as GPIO

class Sys3_SLAM:
    def __init__(self,encoderPins:list[int]) -> None:
        self.X =0 # forward 
        self.Y =0 # left 
        self.direction = 0 # in degrees reletive to the x direction with anticlockwise being positive

        self.LencoderPins = encoderPins[0:1]
        self.RencoderPins = encoderPins[2:3]


    def run(self)->None:
        pass 
    

    def getLocationRot(self)->tuple[float,float,float,bool]:
        return (self.X,self.Y,self.direction,self.home)
    
