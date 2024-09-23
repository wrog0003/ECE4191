# Define clockwise and anti-clockwise as Boolean variables 
ANTICLOCKWISE = False
CLOCKWISE = True

import RPi.GPIO as GPIO # for use of motors 
from math import cos, sin, pi
from time import sleep 

import GLOBALSM1  # used for physical parameters, such as wheel size, wheel base, pins
from ECE4191enums import ACTION # self state 

from SysA_Encoder import SysA_Encoder # get access to the encoder class


class SysB_MotorPos:
    '''This class sets up the motor and encoder pins, and the functions to use them'''
    MINIMUM_SPEED = 20 
    '''Define the minium speed that the robot can do for moving or turning '''
    def __init__(self) -> None:
        '''
        This sets up the motor and encoder pins 
        ### It does not set up pinmode, this should be done in the container class
        '''
        # Set up the motor output pins 
        GPIO.setup(GLOBALSM1.motorLa, GPIO.OUT) 
        GPIO.setup(GLOBALSM1.motorLb, GPIO.OUT)
        GPIO.setup(GLOBALSM1.motorRa, GPIO.OUT)
        GPIO.setup(GLOBALSM1.motorRb, GPIO.OUT)
        # set them up as pwm 
        self.motorLa = GPIO.PWM(GLOBALSM1.motorLa,1000)
        self.motorLb = GPIO.PWM(GLOBALSM1.motorLb,1000)
        self.motorRa = GPIO.PWM(GLOBALSM1.motorRa,1000)
        self.motorRb = GPIO.PWM(GLOBALSM1.motorRb,1000)
        #stop all the motors to prevent robot moving unexpectedly
        self.motorLa.stop()
        self.motorLb.stop()
        self.motorRa.stop()
        self.motorRb.stop()

        # Position control
        self.x_pos = 0
        self.y_pos = 0
        self.rot = 0

        # state control of direction for internal position
        self.state = ACTION.FORWARD

        # position control systems
        self.error_count = 0 
        '''number of extra pulses on the right wheel compared to the left wheel'''
        self.duty_cycle_bias = 0.963 
        '''the fraction of the left wheel duty cycle that should be fed to the right motor'''
        self.EncoderLeft = SysA_Encoder(GLOBALSM1.motorLcha) 
        self.EncoderRight = SysA_Encoder(GLOBALSM1.motorRcha)

        # PI controller variables
        self.duty_cycle = 0 
        '''save the desired duty cycle to for later access'''
        self.right_active_pin = None 
        '''Which pin to change the duty cycle of with the PI controller'''
    
    def forwards(self,duty_cycle:float)->ACTION:
        '''
        This function sets the motors to move forwards at a desired speed. 

        Inputs:
            self: class instance
            duty_cycle: the speed, from 0-100

        Output:
            the internal state of the robot, used for updating the position and rotation
        '''

        # drive the motor forwards 
        self.motorLa.start(0)
        self.motorLb.start(0)
        self.motorRa.start(duty_cycle)
        self.motorRb.start(max(duty_cycle*self.duty_cycle_bias,SysB_MotorPos.MINIMUM_SPEED))
        self.duty_cycle = duty_cycle
        self.RActivePin = self.motorRb

        return ACTION.FORWARD 
    
    def backwards(self, duty_cycle:float)->ACTION:
        '''
        This private function sets the motors to move backwards at a desired speed. 

        Inputs:
            self: class instance
            duty_cycle: the speed, from 0-100

        Output:
            the internal state of the robot, used for updating the position and rotation
        '''
        # duty cycle = controls speed of robot (0 - 100)
        # output: return the state of the robot

        self.motorLa.start(0)
        self.motorLb.start(0)
        self.motorRa.start(duty_cycle)
        self.motorRb.start(max(duty_cycle*self.duty_cycle_bias,SysB_MotorPos.MINIMUM_SPEED))
        self.duty_cycle = duty_cycle
        self.RActivePin = self.motorRa

        return ACTION.BACKWARD 

    def turn(self,duty_cycle:float,clockWise:bool)->ACTION:

        '''
        This function sets the motors to turn left or right at a desired speed

        Inputs:
            self: a class instance
            duty_cycle: the speed, from 0-100
            clockWise: if it should turn clockwise, use False if turing anticlockwise

        Output:
            the internal state of the robot, used for updating the position and rotation
        
        This function checks if it is clockwise or anticlockwise and turns on the correct motors to achieve this movement.
        It also saves which pin of the right motor is active and the desired duty cycle for use in the PI controller.  
        '''
        # input: 
        # duty cycle = controls speed of robot (0 - 100)
        # bool = dirction to turn, clockwise or anticlockwise 
        # output: return the state of the robot

        if clockWise: # turn clockwise 
            self.motorLa.start(0)
            self.motorLb.start(0)
            self.motorRa.start(duty_cycle)
            self.motorRb.start(max(duty_cycle*self.duty_cycle_bias,SysB_MotorPos.MINIMUM_SPEED))
            self.duty_cycle = duty_cycle
            self.RActivePin = self.motorLa
            return ACTION.RIGHT
        else: # turn anti-clockwise
            self.motorLa.start(0)
            self.motorLb.start(0)
            self.motorRa.start(duty_cycle)
            self.motorRb.start(max(duty_cycle*self.duty_cycle_bias,SysB_MotorPos.MINIMUM_SPEED))
            self.duty_cycle = duty_cycle
            self.RActivePin = self.motorRb
            return ACTION.LEFT
        
    def stop(self)->None:
        '''Stops the motors'''
        self.motorLa.stop()
        self.motorLb.stop()
        self.motorRa.stop()
        self.motorRb.stop()

    def EncoderController(self, delL:int, delR:int) -> float:

        '''
        This function acts as a PI controller to ensure that the motor speeds are the same 

        Inputs:
            self: the class instance
            delL: change in left encoder count 
            delR: change in right encoder count 

        Outputs:
            duty_cycle_bias = bias applied to right encoder so that it matches speed of left motor 


        '''
        # Controller Gains (Proportional, Integral, Derivative)
        Kp = 0.375 # 1.4 is good 
        Ki = 0.17
        
        # Desired difference between the left and the right encoder counts when moving forwards. 
        reference = 0
        
        # Calculates a duty cycle bias to apply (Limited between 0.5 and 1)
        self.duty_cycle_bias = max(0.5,min((Kp*(reference-(delR-delL)) - Ki*self.error_count),1.2))
        
        countDifference = (delR-delL) 
        
        self.error_count += countDifference
        

        self.right_active_pin.start(max(self.duty_cycle*self.duty_cycle_bias,SysB_MotorPos.MINIMUM_SPEED))
        return 
    
    def updatePos(self, x_old:float, y_old:float, rot_old:float)->list[float]:
        '''
        This private function updates the internal position of the robot and runs the PI controller

        Inputs:
            self: an instance of this class
            x_old: the old x location (m)
            y_old: the old y location (m)
            rot_old: the old rotation (degrees)

        Outputs:
            new x: updated x location (m)
            new y: updated y location (m)
            new rot: updated rotation (degrees)

        This uses the internal state of the robot and the change in encoder counts to update position. 
        The internal state is defined based on direction calls that determines how the change in positions is updated.
        For forwards and backwards it maintains the rotation, and then uses sin and cos to add to x and y
        For turning it uses the track width to work out the angle change and adds it to the current angle. It maintains the x and y pos
        '''

        # get the number of pulses the encoder has counted
        [newL, dirL, oldL] = self.EncoderLeft.getValues()
        [newR, dirR, oldR] = self.EncoderRight.getValues()

        # calculate the difference in the number of encoder pulses in the time between the last call
        delL = abs(newL-oldL)
        delR = abs(newR-oldR)
        
        #Call the encoder controller method
        self.EncoderController(delL, delR)
        # calculate the average travelled distance 
        distanceAvg = ((delL*GLOBALSM1.distancePerPulse)+(delR*GLOBALSM1.distancePerPulse))/2 

        # reset coordinates and rotation to 0 
        x = 0
        y = 0 
        rot = 0 

        # calculate the change in x, y and angle positions
        dx = distanceAvg*cos(rot_old*pi/180)
        dy = distanceAvg*sin(rot_old*pi/180)
        dtheta = distanceAvg*360/GLOBALSM1.wheelBaseCircumference
        # determine direction that the robot has traveled in the previous timestep & update coordinates
        if self.state == ACTION.FORWARD: 
            rot = rot_old
            y = y_old + dy 
            x = x_old + dx

        elif self.state == ACTION.BACKWARD:
            rot = rot_old
            y = y_old - dy
            x = x_old - dx

        elif self.state == ACTION.LEFT:
            x = x_old
            y = y_old
            rot = rot_old + dtheta

        elif self.state == ACTION.RIGHT:
            x= x_old
            y = y_old
            rot = rot_old - dtheta

        # limit the angle to the correct domain
        if rot > 180:
            rot -=360 
        elif rot <-180:
            rot +=360 
            
        return x,y,rot

    def delay(self,time:float)->None:
        '''
        This delay function sleeps and runs localization.

        Inputs: 
            self: the class instance
            time: the time in seconds to delay, must be greater than 0.02
        
        Functionality:
            This breaks the time up into 0.02 second chunks and runs localization after each chunk. 
            This prevents error buildup. 
        '''
        internalTime = 0
        while internalTime <time: # while waiting
            sleep(0.02) #sleep
            self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot) # update pos and call controller 
            internalTime+=0.02 # increment time 
        return 
    
    def __del__(self)->None:
        '''the function that deletes this class and releases pins'''
        self.stop() 
        GPIO.cleanup() # release motor pins
        del self.EncoderLeft
        del self.EncoderRight

    