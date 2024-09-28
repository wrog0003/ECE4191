from gpiozero import Button # for detecting colltion of balls
import RPi.GPIO as GPIO # for use of motors 

import GLOBALSM1 # for which pins to use 

from time import sleep # to enable delaying


class SysC_BallCollection:
    '''This system deals with detecting the ball count and moving the conveyer'''
    MAXBALLS = 4 
    LOADTIME = 0.5
    UNLOADTIME = 2
    '''The maximum number of balls that the system can hold before it should return to the box'''
    def __init__(self, ConveyerPin: int= GLOBALSM1.servo1, ButtonPin:int = GLOBALSM1.button1) -> None:
        '''Sets up the variables and servo and button pins'''
        self.ballCount = 0
        '''Number of balls in system'''
        self.ballFlag = False
        '''Flag to show that a new ball has entered the system'''
        self.full = False
        '''Flag to indicate that the system is full'''

        #set up servo 
        GPIO.setup(ConveyerPin, GPIO.OUT) # set the pin type
        self.Servo = GPIO.PWM(ConveyerPin,1000) # set a pwm
        '''Servo pin that controls the conveyer'''
        self.Servo.stop() # stop the servo to prevent accidental movement

        #set up the button to detect a ball
        self.BallButton = Button(ButtonPin, pull_up=True) # high = logic 1, low = logic 0 
        '''Button to detect balls entering the system'''
        self.BallButton.when_pressed = self.IncrementBall

    def IncrementBall(self,channel)->None:
        '''Callback function to increment the number of balls'''
        self.ballCount+=1 
    
    def GetBallCount(self)->int:
        '''returns the number of balls in the system'''
        return self.ballCount
    
    def _ResetBallCount(self)->None:
        '''Resets the number of balls in the system'''
        self.ballCount = 0 
    
    def addBallToSystem(self)->bool:
        '''Moves the conveyer enough to lock a ball into the system

        Outputs:    True if the number of balls is the maximum for the system'''

        self.Servo.start(90) #run the conveyer
        sleep(SysC_BallCollection.LOADTIME) # run the conveyer for the correct amount of time 
        self.Servo.stop() # stop the conveyer 
        return self.ballCount >=SysC_BallCollection.MAXBALLS # check if the system is full
    
    def unloadBalls(self)->None:
        '''Unloads the system of balls and resets the ball count'''
        self.Servo.start(90) # run the conveyer
        sleep(SysC_BallCollection.UNLOADTIME) # wait for a full period of the tracks 
        self.Servo.stop() # stop the servo
        self._ResetBallCount() # reset the ball count


    def __del__(self)->None:
        '''Deletes the class instance and releases the pins'''
        self.Servo.stop() # stops the motor to prevent unexpected movement
        self.BallButton.close() # release the pin
        GPIO.cleanup() # may be a double up but releases the conveyer pin


    