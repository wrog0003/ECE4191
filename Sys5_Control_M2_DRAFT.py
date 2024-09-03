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
        #self.vision = Sys4_Vision()

        #Position control

        # define the initial position of the robot as (0,0) with a rotation of 0 
        self.x_pos =0
        self.y_pos = 0
        self.rot = 0 

        self.State = None # State is the movement of the robot i.e. Left, Right, Forward, Backward
        # initalise to none bc robot is not moving

        # Keeps track of difference in encoder pulse counts. 
        self.error_count = 0

        # Apllies a bias to the right motor to increase or decrease its output speed to ensure straight line motion.
        self.duty_cycle_bias = 0.963

        self.EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
        self.EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor

            
    def _forwards(self,duty_cycle:float)->ACTION:
    
        # duty cycle = controls speed of robot (0 - 100)
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1a.start(0)
        self.pwm1b.start(duty_cycle)
        self.pwm2a.start(0)
        self.pwm2b.start(max(duty_cycle*self.duty_cycle_bias,5))

        return ACTION.FORWARD

    def _backwards(self, duty_cycle:float)->ACTION:
    
        # duty cycle = controls speed of robot (0 - 100)
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1b.start(0)
        self.pwm1a.start(duty_cycle)
        self.pwm2b.start(0)
        self.pwm2a.start(max(duty_cycle*self.duty_cycle_bias,5))

        return ACTION.BACKWARD 

    def _turn(self,duty_cycle:float,clockWise:bool)->ACTION:

        # input: 
        # duty cycle = controls speed of robot (0 - 100)
        # bool = dirction to turn, clockwise or anticlockwise 
        # output: return the state of the robot

        if clockWise: # turn clockwise 
            self.pwm1a.start(0)
            self.pwm1b.start(duty_cycle)
            self.pwm2a.start(max(duty_cycle*self.duty_cycle_bias,5))
            self.pwm2b.start(0)
            return ACTION.RIGHT
        else: # turn anti-clockwise
            self.pwm1a.start(duty_cycle)
            self.pwm1b.start(0)
            self.pwm2a.start(0)
            self.pwm2b.start(max(duty_cycle*self.duty_cycle_bias,5))
            return ACTION.LEFT

    def _stop(self)->None: # stop movement of robot 
        self.pwm1a.stop()
        self.pwm1b.stop()
        self.pwm2a.stop()
        self.pwm2b.stop()

    # Manual exit function to prevent loss of pin control
    def _exemptExit(self)->None:
        self._stop()
        GPIO.cleanup()
        self.EncoderL.end()
        self.EncoderR.end()
        #self.vision.disconnect() 
    
    # Release all Pins
    def release(self)->None:
        self._stop()
        #self.vision.disconnect()
        GPIO.cleanup()
        sleep(0.1) # ensure that every peripheral is released 

    # Position tracking 
    def _updatePos(self, x_old:float, y_old:float, rot_old:float)->list[float]:

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
        self.error_count += (delL-delR)
        # calculate the average travelled distance 
        distanceAvg = ((delL*GLOBALSM1.distancePerPulse)+(delR*GLOBALSM1.distancePerPulse))/2 
    
        # reset coordinates and rotation to 0 
        x = 0
        y = 0 
        rot = 0 

        # calculate the change in x, y and angle positions
        dx = distanceAvg*cos(rot*pi/180)
        dy = distanceAvg*sin(rot*pi/180)
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

        return x,y,rot
    
    # Method that takes in the change in encoder pulses on the left and the right encoders and uses a controller to change the duty cycle bias.  
    def EncoderController(self, delL:int, delR:int) -> float:
        # Controller Gains (Proportional, Integral, Derivative)
        Kp = 0
        Ki = 0
        #Kd = 0

        # Desired difference between the left and the right encoder counts when moving forwards. 
        desired_difference = 0

        # Calculates a duty cycle bias to apply 
        self.duty_cycle_bias = min((Kp*(delL-delR) + Ki*self.error_count),1)


        return 

    # fuction that calls the vision system and determines the direction that the robot needs to move 
    # the distance to the ball and if the robot will hit the ball in the next move 
    def hitBallSettings(self) -> Tuple[DIRECTION, float, float, bool]:

        # INPUTS: self 

        # OUTPUTS
        # direction = direction that the robot is relative to the ball
        # speed = speed that the robot should travel at in the next time step
        # pauseTime = how long the robot should travel at this speed for 
        # noHit = whether the ball will be hit by performing this movement 

        (direction, temp, distance)= self.vision.detect() # run vision check 
        noHit = True
        if (direction == DIRECTION.Ahead):

            # settings if robot if more than 0.55m away from the ball 
            speed = 30 # set the drive speed 
            pauseTime = 5 # how long in secs the robot should drive forwards for 

            if (distance <  0.55): # if robot is less than 0.55 m from ball 
                speed = 20 # reduce speed of robot
            
            if (distance < 0.40): # close to ball, drive forwards until you hit it
                speed = 30 
                pauseTime = 5.5 
                noHit = False

        elif (direction == DIRECTION.CannotFind):
            speed = 20
            pauseTime = 0.3

        else: # ball is in field of view but is either left or right
            speed = 15
            pauseTime = 0.15 
        
        return direction, speed, pauseTime, noHit
    
    # function that implements the movement to hit the ball
    def hitBall(self) -> None: 
        noHit = True 
        try:
            while(noHit): # while not close enough to the ball

                # get position of robot 
                self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot)
                
                # get the speed, pauseTime and if the robot will hit the ball in the next timestep
                direction, speed, pauseTime, noHit = self.hitBallSettings() 

                # Ball AHEAD
                if (direction == DIRECTION.Ahead):
                    self.State = self._forwards(speed)
                
                # Ball CANNOT FIND 
                elif (direction == DIRECTION.CannotFind):
                    self.State = self._turn(speed, ANTICLOCKWISE)
                
                # Ball LEFT 
                elif (direction == DIRECTION.Left):
                    self.State = self._turn(speed,ANTICLOCKWISE)
                
                # Ball RIGHT
                else:
                    self.State = self._turn(speed,CLOCKWISE)
                
                sleep(pauseTime) # do that movement for the designated n.o sec defined in PauseTime
                self._stop() # stop movement of robot temporarily until next action is determined
            
            print(f'Reached {self.x_pos}, {self.y_pos} with rot of {self.rot}\n')

        
        except KeyboardInterrupt: 
            self._exemptExit()
        
    #disengages from the ball so that it doesn't hit the ball incorrectly during the return to home sequence
    def disEngage(self)->None:
        
        speed = 30 # set speed
        self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot) # determine OG pos

        # Backs away from the ball for 3 seconds
        self.State = self._backwards(speed)
        sleep(3) 
        self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot)

        # Re aligns the dolly wheel
        self.State = self._forwards(speed)
        sleep(0.5) 
        self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot)

    # calculates the number of pulses required to achieve the desired turn and forwards direction
    def EncoderPulseCalulator(self, angle:float, forward_distance:float) -> list[int]:
        # INPUTS
        # self 
        # angle = desired angle rotation 
        # forward_distance = required forward distance

        # OUTPUTS
        # angle_numPulses = number of angle pulse required to achieve desired rotation
        # forward_numPulses = number of pulses required to achieve desired forwards distance

    
        # calculate the number of pulses required to achieve the turn 
        angle_distance = GLOBALSM1.wheelBaseCircumference*abs(angle)/360
        angle_numPulses = (angle_distance/GLOBALSM1.distancePerPulse) + self.EncoderL.encoderCount

        # calculate the number of pulses required to achieve the distance required 
        forward_numPulses = (forward_distance/GLOBALSM1.distancePerPulse) + self.EncoderL.encoderCount

        return angle_numPulses, forward_numPulses

    
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
                self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos, self.y_pos, self.rot)
                sleep(0.02)

            self._stop()

            # drive forwards until you reach desired forwards distance 

            self.State = self._forwards(forward_speed)

            while (self.EncoderL.encoderCount <forward_numPulses):# keep going fowards until you reach the desired number of pulses 
                self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot)
                sleep(0.02)

            self._stop()
            GPIO.cleanup()

        except KeyboardInterrupt:
            self._exemptExit()
    

    def Home(self) -> None:
        try:
            speed = 30 # speed at which robot will move on its journey home
            print(f' at X {self.x_pos}, Y {self.y_pos}, rot {self.rot}\n')

            # calculate angle to return home
            angle = atan2(-self.y_pos,-self.x_pos)*180/pi
            angle = angle - self.rot # make it relative to the current position of the robot

            # calculate forward distance required to return home
            forward_distance = sqrt(self.x_pos**2 + self.y_pos**2)

            # calculate the number of pulses required to achieve the turn and the forward distance 
            angle_numPulses, forward_numPulses = self.EncoderPulseCalulator(angle, forward_distance)

            forward_numPulses = forward_numPulses*1.1 # add an offset to account for accumualted inaccuracies

            self.turnGoForwards(speed, speed, angle, angle_numPulses,forward_numPulses) # make the robot turn and drive forward this distance 

        except KeyboardInterrupt:
            self._exemptExit()
        

    def searchPattern(self)-> None:

        # define speed constants 
        turn_speed = 15
        forward_speed = 30
        self.x_pos, self.y_pos, self.rot = self._updatePos(self.x_pos,self.y_pos,self.rot) # update position

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
                    angle_numPulses, forward_numPulses = self.EncoderPulseCalulator(angle, forward_distance)
                    
                    self.turnGoForwards(turn_speed, forward_speed, angle, angle_numPulses,forward_numPulses) # make the robot turn and drive forward this distance 

                    #Print out location reached based on encoders
                    print(f'Reached {self.x_pos}, {self.y_pos} with rot of {self.rot}\n')

        except KeyboardInterrupt:
            self._exemptExit() 



if __name__ == "__main__":
    robot = Sys5_Control() 
    #robot.vision.tolerence = 25
    # tell robot to do stuff between here 
    #robot.searchPattern()
    #robot.hitBall()
    #robot.disEngage()
    #robot.Home()

    [angle_numPulses, forward_numPulses] = robot.EncoderPulseCalulator(0, 3)
    robot.turnGoForwards(90, 90, 0, angle_numPulses, forward_numPulses)
            
    print(robot.error_count)
    print(f'Finished {robot.x_pos}, {robot.y_pos} with rot of {robot.rot}\n') 
    
    #and here 
    robot.release() #release motor pins 







