from enum import Enum

class STATE (Enum):
    unloading = 1
    return2Home = 2 #use for milestone 1
    turnAround =3 
    turn2Ball = 4 # use for milestone 1
    move2Ball = 5 # use for milestone 1 

class DIRECTION (Enum):
    Left =1
    Ahead = 2
    Right = 3
    CannotFind = 4 
    
    