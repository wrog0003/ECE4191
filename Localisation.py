#This class if used to know the location of the robot at any time 
# it should feed into the Control class and instantiate 2 encoder classes 
#Define X as the forward direction based on the robot start position 
#Define Y as right releteive to the robot start point 
from math import pi

class Localisation:
    #radius should be in mm 
    def __init__(self,radius:float) -> None:
        #radius should be in mm 
        self.X = 0
        self.Y = 0
        self.Rot = 0 
        self.motorLEncoder = ssss 
        self.motorREncoder = ssss
        self.oldLpulses = 0
        self.oldRpulses = 0
        self.circumference = radius*2*pi

    #updates localisation, should be called at a high frequency to ensure that the localisation is correct 
    def updatePos(self)->None:
        newPulseL = self.motorLEncoder.getPulses()
        newPulseR = self.motorREncoder.getPulses()
        dirL = self.motorLEncoder.getDirection()
        dirR = self.motorREncoder.getDirection() 
        delLpulses = newPulseL-self.oldLpulses
        delRpulses = newPulseR-self.oldRpulses
        # check if forwards or backwards
        if dirL == dirR:
            # check if forward 

    #data that the control system cares about 
    def getPos(self)-> list[float]:
        return [self.X,self.Y]
    
    def getRot(self)->float:
        return self.Rot 
