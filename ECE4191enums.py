# Enums for readibility by Warren Rogan

'''
### The collection of enums that define the state, ball detected direction and current action of the ECE4191 Robot
'''
from enum import Enum

class STATE (Enum):
    '''
    This Enum allows for easy reference of the state the the robot is in
    '''
    unloading = 1
    return2Home = 2 #use for milestone 1
    turnAround =3 
    turn2Ball = 4 # use for milestone 1
    move2Ball = 5 # use for milestone 1 
    atHome = 6
    null = 7

class DIRECTION (Enum):
    '''
    This Enum defines the direction that a detected ball is in
    '''
    Left =1
    Ahead = 2
    Right = 3
    CannotFind = 4 

class ACTION (Enum):
    '''
    This Enum defines the internal state of the robot in the context of the direction is is traveling in
    '''
    FORWARD =1
    BACKWARD =2
    LEFT = 3
    RIGHT =4

    
    