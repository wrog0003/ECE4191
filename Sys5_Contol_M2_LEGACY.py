# overall brains script for robot, written by Emma Vladicic and Warren Rogan 

from ECE4191enums import STATE, DIRECTION, ACTION
from Motor_code.encoderClass import SimpleEncoder
from Sys4_Vision import Sys4_Vision

from time import sleep
from math import pi, atan2, sqrt, sin, cos

from typing import Tuple

import RPi.GPIO as GPIO
import GLOBALSM1 # for physical values of the robot i.e. wheel diameter, distance b/w wheeels etc   

# Define important variables 

# Define clockwise and anti-clockwise as Boolean variables 
ANTICLOCKWISE = False
CLOCKWISE = True

# Define physical GPIO pins on the Rpi 
# Motor Left (to drive motors)
motor1a = 17
motor1b = 27

# Motor Right (to drive motors)
motor2a = 23
motor2b = 24

# Encoder Left (to recieve data from encoders)
motor1cha = 13
motor1chb = 19

# Encoder Right (to recieve data from encoders)
motor2cha = 5
motor2chb = 6

# Wheel bias (used to callibrate differences in wheel movement)
#duty_cycle_bias = 0.963 


#Milestone 1 control top level class 

class Sys5_Control:

    # initialisation of the class
    def __init__(self) -> None:
        '''
        This setups up the variables needed to make an instance of the class
        It completes the following:

            1. Set GPIO pinmode 
            2. Set GPIO motor pin types
            3. Sets motor pins as PWM instance variables
            4. Creates instance variables
                4.0. for Vision class instance
                4.1. for position and rotation
                4.2. for internal state
                4.3. to keep track of encoder mismatch count
                4.4. and defines to define starting motor bias
                4.5. 2 instances of the encoder class
                4.6. an active motor PWM pin and desired duty cycle copy
        '''

        GPIO.setmode(GPIO.BCM) # set pin types 

        # Set up the motor output pins 
        GPIO.setup(motor1a, GPIO.OUT) 
        GPIO.setup(motor1b, GPIO.OUT)
        GPIO.setup(motor2a, GPIO.OUT)
        GPIO.setup(motor2b, GPIO.OUT)

        # Set up the PWM pins
        self.pwm1a = GPIO.PWM(motor1a,1000)
        self.pwm1b = GPIO.PWM(motor1b,1000)
        self.pwm2a = GPIO.PWM(motor2a,1000)
        self.pwm2b = GPIO.PWM(motor2b,1000)
        self._stop() # prevent random movements
        
        # initalise the vision system
        self.vision = Sys4_Vision()

        #Position control
        # define the initial position of the robot as (0,0) with a rotation of 0 
        self.x_pos =0
        self.y_pos = 0
        self.rot = 0 

        self.State = None # State is the movement of the robot i.e. Left, Right, Forward, Backward
        '''The current state of the robot, Forwards, Backward, Left, Right or None'''
        # initalise to none bc robot is not moving

        # Keeps track of difference in encoder pulse counts. 
        self.error_count = 0

        # Apllies a bias to the right motor to increase or decrease its output speed to ensure straight line motion.
        self.duty_cycle_bias = 0.963

        self.EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
        self.EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor

        #PI controller access variables
        self.duty_cycle = 0  # allows access for the PI controller 
        self.RActivePin = None # allows correct motor access to the controller 

        # Self variables to keep track of the number of balls collected and the capacity of the conveyor storage. 
        self.capacity = 3 # Maximum number of tennis balls that can be stored in the conveyor system. 
        self.numBalls = 0 # Number of balls collected by the robot on any given run.   

            
    def _forwards(self,duty_cycle:float)->ACTION:
        '''
        This private function sets the motors to move forwards at a desired speed. 

        Inputs:
            self: class instance
            duty_cycle: the speed, from 0-100

        Output:
            the internal state of the robot, used for updating the position and rotation
        '''
        # duty cycle = controls speed of robot (0 - 100)
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1a.start(0)
        self.pwm2a.start(0)
        self.pwm1b.start(duty_cycle)
        self.pwm2b.start(max(duty_cycle*self.duty_cycle_bias,5))
        self.duty_cycle = duty_cycle
        self.RActivePin = self.pwm2b


        return ACTION.FORWARD

    def _backwards(self, duty_cycle:float)->ACTION:
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

        self.pwm1b.start(0)
        self.pwm2b.start(0)
        self.pwm1a.start(duty_cycle)
        self.pwm2a.start(max(duty_cycle*self.duty_cycle_bias,5))
        self.duty_cycle = duty_cycle
        self.RActivePin = self.pwm2a

        return ACTION.BACKWARD 

    def _turn(self,duty_cycle:float,clockWise:bool)->ACTION:

        '''
        This private function sets the motors to turn left or right at a desired speed

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
            self.pwm1a.start(0)
            self.pwm2b.start(0)
            self.pwm1b.start(duty_cycle)
            self.pwm2a.start(max(duty_cycle*self.duty_cycle_bias,5))
            self.duty_cycle = duty_cycle
            self.RActivePin = self.pwm2a
            return ACTION.RIGHT
        else: # turn anti-clockwise
            self.pwm1b.start(0)
            self.pwm2a.start(0)
            self.pwm1a.start(duty_cycle)
            self.pwm2b.start(max(duty_cycle*self.duty_cycle_bias,5))
            self.duty_cycle = duty_cycle
            self.RActivePin = self.pwm2b
            return ACTION.LEFT

    def _stop(self)->None: # stop movement of robot
        '''
        This private function stops all the motors to prevent unplanned movement.
        ''' 
        self.pwm1a.stop()
        self.pwm1b.stop()
        self.pwm2a.stop()
        self.pwm2b.stop()
        # does not include a state update as the robot may continue moving for a short period as it has interia

    # Manual exit function to prevent loss of pin control
    def _exemptExit(self)->None:
        '''
        This private function is used to allow for manual exit of code if needed by disconnecting from peripherals.

        Inputs: 
            self: class instance

        It stops the motors, disconnects the motor GPIO pins, disconnects the encoders pins, and disconnects the camera feed. 
        '''
        self._stop()
        GPIO.cleanup()
        self.EncoderL.end()
        self.EncoderR.end()
        self.vision.disconnect() 
    
    # Release all Pins
    def release(self)->None:
        '''
        This function disconnects from connected peripherals to prevent the devices from preventing reconnection.

        It stops the motors, disconnects the camera and disconnects the GPIO
        '''
        self._stop()
        self.vision.disconnect()
        GPIO.cleanup()
        sleep(0.1) # ensure that every peripheral is released 

    # Position tracking 
    def _updatePos(self, x_old:float, y_old:float, rot_old:float)->list[float]:
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

        # inputs 
        # self = direction that the robot is moving 
        # x_old, y_old, rot_old = previous x, y coordinate and rotation of robot

        # output 
        # x, y, rot = current x, y  coordinates and rotation of robot

        # get the number of pulses the encoder has counted
        [newL, dirL, oldL] = self.EncoderL.getValues()
        [newR, dirR, oldR] = self.EncoderR.getValues()

        # calculate the difference in the number of encoder pulses in the time between the last call
        delL = abs(newL-oldL)
        delR = abs(newR-oldR)
        #print(delL-delR)
        
        #Call the encoder controller method
        #if self.State == ACTION.FORWARD or self.State ==ACTION.BACKWARD:
        self.EncoderController(delL, delR)
        #print(self.duty_cycle_bias)
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
        # determine direction that the robot has travelled in the previous timestep & update coordinates
        if self.State == ACTION.FORWARD: 
            rot = rot_old
            y = y_old + dy 
            x = x_old + dx

        elif self.State == ACTION.BACKWARD:
            rot = rot_old
            y = y_old - dy
            x = x_old - dx

        elif self.State == ACTION.LEFT:
            x = x_old
            y = y_old
            rot = rot_old + dtheta

        elif self.State == ACTION.RIGHT:
            x= x_old
            y = y_old
            rot = rot_old - dtheta

        # limit the angle to the correct domain
        if rot > 180:
            rot -=360 
        elif rot <-180:
            rot +=360 
        #print(f'x{x},y{y},rot{rot}')
        return x,y,rot
    
    #Improved sleep function to maintain localization and PI control 
    def _delay(self,time:float)->None:
        '''
        This private delay function sleeps and runs localization.

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


    # Method that takes in the change in encoder pulses on the left and the right encoders and uses a controller to change the duty cycle bias.  
    def EncoderController(self, delL:int, delR:int) -> float:

        '''
        This function acts as a PI controller to ensure that the motor speeds are the same 

        Inputs:
            self: the class instance
            delL: change in left encoder count 
            delR: change in right encoder count 

        Outputs:
            duty_cycle_bias = biad applied to right encoder so that it matches speed of left motor 


        '''
        # Controller Gains (Proportional, Integral, Derivative)
        Kp = 0.375 # 1.4 is good 
        Ki = 0.17
        
        # Desired difference between the left and the right encoder counts when moving forwards. 
        reference = 0
        
        # Calculates a duty cycle bias to apply (Limited between 0.5 and 1)
        self.duty_cycle_bias = max(0.5,min((Kp*(reference-(delR-delL)) - Ki*self.error_count),1.2))
        
        countDifference = (delR-delL) 
        #print(f'duty cycle {self.duty_cycle_bias}')
        self.error_count += countDifference
        #print(f'err accum = {self.error_count}')

        self.RActivePin.start(max(self.duty_cycle*self.duty_cycle_bias,5))
        return 

    def hitBallSettings(self) -> Tuple[DIRECTION, float, float, bool]:
        '''
        Calls the vision system and determines the direction that the robot needs to move 
        the distance to the ball and if the robot will hit the ball in the next move 

        INPUTS
            self: the class instance

        OUTPUTS
            direction = direction that the robot is relative to the ball
            speed = speed that the robot should travel at in the next time step
            pauseTime = how long the robot should travel at this speed for 
            noHit = whether the ball will be hit by performing this movement 

        '''

        (direction, temp, distance)= self.vision.detect() # run vision check 
        noHit = True
        if (direction == DIRECTION.Ahead):

            # settings if robot if more than 0.55m away from the ball 
            speed = 70 # set the drive speed 
            pauseTime = 0.5 # how long in secs the robot should drive forwards for 

            if (distance <  0.55): # if robot is less than 0.55 m from ball 
                speed = 50 # reduce speed of robot
            
            if (distance < 0.40): # close to ball, drive forwards until you hit it
                speed = 50 
                pauseTime = 4
                noHit = False

        elif (direction == DIRECTION.CannotFind):
            speed = 30
            pauseTime = 0.1

        else: # ball is in field of view but is either left or right
            speed = 15
            pauseTime = 0.1
        
        return direction, speed, pauseTime, noHit
    
    
    def hitBall(self) -> None: 
        '''
        gets the robot to hit the ball once it is close enough

        Inputs: 
            self: the class instance 
        
        Outputs:
            none
        '''
        noHit = True 
        try:
            while(noHit): # while not close enough to the ball

                # get the speed, pauseTime and if the robot will hit the ball in the next timestep
                direction, speed, pauseTime, noHit = self.hitBallSettings() 

                # Ball AHEAD
                if (direction == DIRECTION.Ahead):
                    self.State = self._forwards(speed)
                
                # Ball CANNOT FIND 
                elif (direction == DIRECTION.CannotFind):
                    self._stop()
                    self.State = self._turn(speed, ANTICLOCKWISE)
                
                # Ball LEFT 
                elif (direction == DIRECTION.Left):
                    self._stop()
                    self.State = self._turn(speed,ANTICLOCKWISE)
                
                # Ball RIGHT
                else:
                    self._stop()
                    self.State = self._turn(speed,CLOCKWISE)
                
                self._delay(pauseTime)# do that movement for the designated n.o sec defined in PauseTime
                self._stop() # stop movement of robot temporarily until next action is determined
                # get position of robot 
                self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot)
            
            print(f'Reached {self.x_pos}, {self.y_pos} with rot of {self.rot}\n')

        
        except KeyboardInterrupt: 
            self._exemptExit()
        
    def disEngage(self)->None:

        '''
        Disengages from the ball so that it doesn't hit the ball incorrectly during the return to home sequence

        Inputs: 
            self: the class instance 

        Outputs:
            none 
        '''
        
        speed = 30 # set speed
        self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot) # determine OG pos

        # Backs away from the ball for 3 seconds
        self.State = self._backwards(speed)
        self._delay(3) 
        

        # Re aligns the dolly wheel
        self.State = self._forwards(speed)
        self._delay(0.5) 
        

    def lineFoundResponse (self):
        
        '''
        Controls the respone if the Vision System detects a line/boundary 
        Inputs:
            self: the class instance 

        Outputs:
            none 
        '''

        # set angle of rotation 
        angle = 120*(180/pi) # 120 degree rotation CCW radians 

        speed = 30 # set a speed for rotation

        # make the robot turn this angle
        self.turnAngle(speed, angle)

        return 

    # calculates the number of pulses required to achieve the desired turn and forwards direction
    def EncoderPulseCalulator(self, angle:float, forward_distance:float) -> list[int]:
        '''
        Calculates the number of pulses required to achieve the desired turn and forwards direction
        INPUTS:
            self: the class instance 
            angle = desired angle rotation 
            forward_distance = required forward distance

        OUTPUTS:
            angle_numPulses = number of angle pulse required to achieve desired rotation
            forward_numPulses = number of pulses required to achieve desired forwards distance
        '''

        # calculate the number of pulses required to achieve the turn 
        angle_distance = GLOBALSM1.wheelBaseCircumference*abs(angle)/360
        angle_numPulses = (angle_distance/GLOBALSM1.distancePerPulse) + self.EncoderL.encoderCount

        # calculate the number of pulses required to achieve the distance required 
        forward_numPulses = (forward_distance/GLOBALSM1.distancePerPulse) + self.EncoderL.encoderCount

        return angle_numPulses, forward_numPulses

    # takes in an angle and makes the robot turn that angle 
    def turnAngle(self, speed:int, angle:float)-> None:
        '''
        takes in an angle and makes the robot turn that angle 
        
        INPUTS: 
            self: the class instance
            speed: speed at which the wheels turn (0-100)
            angle: desired angle of rotatiion

        OUTPUTS:
            none 

        '''
        try: 
            # set forward distance to zero as we are only rotating 
            forward_distance = 0 
            
            # calculte the number of pulses need to achieve the angle turn 
            angle_numPulses, _ = self.EncoderPulseCalulator(angle, forward_distance)

            # rotate to achieve the desired angle 
            if (angle >-1 and angle <1): # no rotation required 
                sleep(0.01)

            elif (angle>0):
                self.State = self._turn(speed,ANTICLOCKWISE) # rotate CCW

            else: 
                self.State = self._turn(speed,CLOCKWISE) # rotate CW 

            while (self.EncoderL.encoderCount < angle_numPulses):
                self._delay(0.02)

            self._stop()
            #GPIO.cleanup() shouldn't clean up at this point

        except  KeyboardInterrupt: 
            self._exemptExit()

    # takes in a forward distance and the robot goes forwards for that distance 
    def forwardsDistance(self, speed:int, forward_distance:float)-> None: 
        '''
        takes in a distance and makes the robot go forwards that distance
        
        INPUTS: 
            self: the class instance
            speed: speed at which the wheels turn (0-100)
            angle: desired distance to travel forwards

        OUTPUTS:
            none 
        '''

        try:
            # set angle to zero as we are only driving forwards 
            angle = 0 
            
            # calculte the number of pulses need to achieve the angle turn 
            _,forward_numPulses = self.EncoderPulseCalulator(angle, forward_distance)
            # drive forwards until you reach desired forwards distance 

            self.State = self._forwards(speed)

            while (self.EncoderL.encoderCount <forward_numPulses):# keep going fowards until you reach the desired number of pulses 
                self._delay(0.02)

            self._stop()
            GPIO.cleanup()

        except KeyboardInterrupt:
            self._exemptExit()

    # LEGACY FUNTION DO NOT USE!!!
    def turnGoForwards(self, turn_speed:int, forward_speed:int, angle:float, angle_numPulses:float, forward_numPulses:float)-> None:
        try: 
            # rotate to achieve the desired angle 
            if (angle >-1 and angle <1): # no rotation required 
                sleep(0.01)

            elif (angle>0):
                self.State = self._turn(turn_speed,ANTICLOCKWISE) # rotate CCW

            else: 
                self.State = self._turn(turn_speed,CLOCKWISE) # rotate CW 

            while (self.EncoderL.encoderCount < angle_numPulses):
                self._delay(0.02)

            self._stop()

            # drive forwards until you reach desired forwards distance 

            self.State = self._forwards(forward_speed)

            while (self.EncoderL.encoderCount <forward_numPulses):# keep going fowards until you reach the desired number of pulses 
                self._delay(0.02)

            self._stop()
            

        except KeyboardInterrupt:
            self._exemptExit()
    

    def Home(self) -> None:

        '''
        makes the robot return to its starting position (i.e. home)
        
        INPUTS: 
            self: the class instance

        OUTPUTS:
            none 
        '''
        try:
            speed = 30 # speed at which robot will move on its journey home
            print(f' at X {self.x_pos}, Y {self.y_pos}, rot {self.rot}\n')

            # calculate angle to return home
            angle = atan2(-self.y_pos,-self.x_pos)*180/pi
            angle = angle - self.rot # make it relative to the current position of the robot

            # calculate forward distance required to return home
            forward_distance = sqrt(self.x_pos**2 + self.y_pos**2)

            # calculate the number of pulses required to achieve the turn and the forward distance 
            #angle_numPulses, forward_numPulses = self.EncoderPulseCalulator(angle, forward_distance)

            #forward_numPulses = forward_numPulses*1.1 # add an offset to account for accumualted inaccuracies

            #self.turnGoForwards(speed, speed, angle, angle_numPulses,forward_numPulses) # make the robot turn and drive forward this distance
            
            print(f'State:{self.State.name} ')
            # turn 
            self.turnAngle(speed, angle)
            print(f'State:{self.State.name} ')

            # go forwards
            self.forwardsDistance(speed, forward_distance)
            print(f'State:{self.State.name} ')

        except KeyboardInterrupt:
            self._exemptExit()
        

    def searchPattern(self)-> None:
        '''
        if the robot cannot find the ball after having turned 90 degrees, it will being a search 
        pattern so that it gets in its field of view
        
        INPUTS: 
            self: the class instance

        OUTPUTS:
            none 
        '''

        # define speed constants 
        turn_speed = 15
        forward_speed = 30
        #self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot) # update position

        try: 
            (direction, temp, distance)= self.vision.detect() # run vision check 

            while(direction == DIRECTION.CannotFind):
                # turn anticlocwise, +ve in our coordinate system for 0.2 seconds 
                self.State = self._turn(turn_speed, ANTICLOCKWISE) 
                sleep(0.2)
                self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot) # update position 

                if (self.rot >= 90): # if we have rotated more than 90 degrees 
                    angle = - self.rot 
                    forward_distance = 2 

                    # call function that determines the number of pulses required to achieve desired movement 
                    #angle_numPulses, forward_numPulses = self.EncoderPulseCalulator(angle, forward_distance)
                    
                    #self.turnGoForwards(turn_speed, forward_speed, angle, angle_numPulses,forward_numPulses) # make the robot turn and drive forward this distance 

                    self.turnAngle(turn_speed, angle)
                    self.forwardsDistance(forward_speed,forward_distance)
                    #Print out location reached based on encoders
                    print(f'Reached {self.x_pos}, {self.y_pos} with rot of {self.rot}\n')

        except KeyboardInterrupt:
            self._exemptExit() 
    
    # Method to use the vision system to find the box and return to it. 
    def toBox(self) -> None:
        pass

    # Method to keep track of the number of balls in the conveyor. Will call the return to home and deposit function once capacity is full. 
    def ballsCollectedTracker(self) -> None:
        # Called everytime a ball is collected and stored in the conveyor. 
        # Could potentially be called from an interrupt on a sensor that detects a ball in the claw or just called in the sequential code logic. 
        self.numBalls += 1 #increment number of balls collected. 

        if self.numBalls < self.capacity: # Robot can continue searching for another ball.
            self.searchPattern() # Locate a ball using a search pattern
            self.hitBall() # MWill move to collect the ball 

        else: # Robot is at capacity and must go to box to deposit the balls
            self.toBox()
            


        



if __name__ == "__main__":
    robot = Sys5_Control() 
    #robot.vision.tolerence = 25
    # tell robot to do stuff between here 
    robot.searchPattern()
    robot.hitBall()
    # robot.disEngage()
    # robot.Home()

    # [angle_numPulses, forward_numPulses] = robot.EncoderPulseCalulator(0, 5)
    # robot.turnGoForwards(70, 70, 0, angle_numPulses, forward_numPulses)
            
    print(robot.error_count)
    print(f'Finished {robot.x_pos}, {robot.y_pos} with rot of {robot.rot}\n') 
    
    #and here 
    robot.release() #release motor pins 