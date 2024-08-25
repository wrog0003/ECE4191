# overall brains script for robot, written by Emma Vladicic and Warren Rogan 

from ECE4191enums import STATE, DIRECTION, ACTION
from Motor_code.encoderClass import SimpleEncoder
from Sys4_Vision import Sys4_Vision

from time import sleep
from math import pi, atan2, sqrt, sin, cos

import RPi.GPIO as GPIO
import GLOBALSM1 #for physical values 

#DEFINE
ANTICLOCKWISE = False
CLOCKWISE = True

#PINS
# Motor Left 
motor1a = 17
motor1b = 27

# Motor Right 
motor2a = 23
motor2b = 24

# Encoder Left 
motor1cha = 13
motor1chb = 19

# Encoder Right 
motor2cha = 5
motor2chb = 6

#Wheel bias
duty_cycle_bias = 0.95


#Milestone 1 controll top level class 
class Sys5_Control:

    def __init__(self) -> None:

        GPIO.setmode(GPIO.BCM) # set pin types 
        # Set up the output pins 
        GPIO.setup(motor1a, GPIO.OUT) 
        GPIO.setup(motor1b, GPIO.OUT)
        GPIO.setup(motor2a, GPIO.OUT)
        GPIO.setup(motor2b, GPIO.OUT)

        # Set up the PWM pins
        self.pwm1a = GPIO.PWM(motor1a,1000)
        self.pwm1b = GPIO.PWM(motor1b,1000)
        self.pwm2a = GPIO.PWM(motor2a,1000)
        self.pwm2b = GPIO.PWM(motor2b,1000)

        # create class instances 
        # Ball finding 
        self.vision = Sys4_Vision()

        #Position control
        self.x_pos =0
        self.y_pos = 0
        self.rot =0 
        self.State = None 
        self.EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
        self.EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor
    
    def _forwards(self,duty_cycle:float)->ACTION:
    
        # input: duty cycle between 0 - 100
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1a.start(0)
        self.pwm1b.start(duty_cycle)
        self.pwm2a.start(0)
        self.pwm2b.start(max(duty_cycle*duty_cycle_bias,5))

        return ACTION.FORWARD

    def _backwards(self,duty_cycle:float)->ACTION:
    
        # input: duty cycle between 0 - 100
        # output: return the state of the robot

        # drive the motor forwards 
        self.pwm1b.start(0)
        self.pwm1a.start(duty_cycle)
        self.pwm2b.start(0)
        self.pwm2a.start(max(duty_cycle*duty_cycle_bias,5))

        return ACTION.BACKWARD 

    def _turn(self,duty_cycle:float,clockWise:bool)->ACTION:

        # input: duty cycle between 0 - 100, if you want to turn clockwise or anticlockwise 
        # output: return the state of the robot

        if clockWise:
            self.pwm1a.start(0)
            self.pwm1b.start(duty_cycle)
            self.pwm2a.start(max(duty_cycle*duty_cycle_bias,5))
            self.pwm2b.start(0)
            return ACTION.RIGHT
        else:
            self.pwm1a.start(duty_cycle)
            self.pwm1b.start(0)
            self.pwm2a.start(0)
            self.pwm2b.start(max(duty_cycle*duty_cycle_bias,5))
            return ACTION.LEFT

    def _stop(self)->None:
        self.pwm1a.stop()
        self.pwm1b.stop()
        self.pwm2a.stop()
        self.pwm2b.stop()

    #Manual exit function to prevent loss of pin control
    def _exemptExit(self)->None:
        self._stop()
        GPIO.cleanup()
        self.EncoderL.end()
        self.EncoderR.end()
    
    #Position tracking 
    def _updatePos(self)->list[float]:
        # get data
        [newL, dirL, oldL] = self.encoderL.getValues()
        [newR, dirR, oldR] = self.encoderR.getValues()

        #difference
        delL = abs(newL-oldL)
        delR = abs(newR-oldR)
        #print(delL-delR)
        #get average travelled distance 
        distanceAvg = ((delL*GLOBALSM1.distancePerPulse)+(delR*GLOBALSM1.distancePerPulse))/2 
        x = 0
        y = 0 
        rot = 0 
        #determine direction
        if self.State == ACTION.FORWARD:
            rot = self.rot
            y= self.y_pos+(distanceAvg*sin(rot*pi/180))
            x = self.y_pos+(distanceAvg*cos(rot*pi/180))
        elif self.State ==ACTION.BACKWARD:
            rot = self.rot
            y= self.y_pos-(distanceAvg*sin(rot*pi/180))
            x = self.x_pos-(distanceAvg*cos(rot*pi/180))
        elif self.State == ACTION.LEFT:
            x= self.x_pos
            y = self.y_pos
            delAngle = distanceAvg*360/GLOBALSM1.wheelBaseCircumference
            rot = self.rot+delAngle
        elif self.State == ACTION.RIGHT:
            x= self.x_pos
            y = self.y_pos 
            delAngle = distanceAvg*360/GLOBALSM1.wheelBaseCircumference
            rot = self.rot-delAngle
        return x,y,rot
    #Calibration test to check that the robot goes in the right direction
    def CalibrationTest(self)->None:
        try:
            self._forwards(30)
            sleep(1)
            self._backwards(30)
            sleep(1)
            self._turn(30,ANTICLOCKWISE)
            sleep(1)
            self._turn(30,CLOCKWISE)
            sleep(1)
            self._stop() 
            print("Test run")
            GPIO.cleanup()
    # clear up if exited manually
        except KeyboardInterrupt:
            self._exemptExit()
    #release pins
    def release(self)->None:
        GPIO.cleanup()
    #Simple goto absolute from starting point 
    def GoTo(self,X:float,Y:float,speed:float = 30)->None:
        
        # get angle to point
        angle = atan2(Y,X)*180/pi -self.rot 
        distance = GLOBALSM1.wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
        numPulses = distance/GLOBALSM1.distancePerPulse+self.EncoderL.encoderCount
        try:
            # rotate 
            if (angle >-1 and angle <1): # no rotation required 
                sleep(0.01)
            elif (angle>0):
                self.State= self._turn(speed,False)# rotate CCW
            else: 
                self.State=self._turn(speed,True)# rotatte CW 
            while (self.EncoderL.encoderCount <numPulses):
                self.x_pos, self.y_pos, self.rot = self._updatePos()
                sleep(0.02)
            self._stop()

            distance = sqrt((X-self.x_pos)**2+(Y-self.y_pos)**2) # calculate distance to drive forward 
            encoderOldCount = self.EncoderL.encoderCount # update encoder count 
            numPulses = (distance/GLOBALSM1.distancePerPulse)+encoderOldCount # get the new final target pulses
            self.State = self._forwards(speed) # drive forwards 
            while (self.EncoderL.encoderCount <numPulses):# keep going forwards until you reach the desired number of pulses 
                self.x_pos, self.y_pos, self.rot = self._updatePos()
                sleep(0.02)
            self._stop()
            #Print out location reached based on encoders
            self.x_pos, self.y_pos, self.rot = self._updatePos()
            print(f'Reached {self.x_pos}, {self.y_pos} with rot of {self.rot}\n')

        except KeyboardInterrupt:
            self._exemptExit()
    #Ball hitting functions 
    def hitBallBasic(self)->None:
        pauseTime = 0.2
        noHit = True
        try :
            while (noHit): # while not close enough to ball 
                (direction, temp, distance)= self.vision.detect() # run vision check 
                if (direction == DIRECTION.Ahead): # if ball ahead
                    speed = 100
                    pauseTime = 0.5
                    if (distance <0.4):
                        speed = 20
                        self.vision.tolerence = 100
                        self.State = self._forwards(speed)
                    if (distance <0.3): # if close to ball 
                        self.State = self._forwards(30)
                        sleep(4)
                        self.x_pos, self.y_pos, self.rot = self._updatePos()
                        noHit = False # end 
                        self._stop()
                    else: 
                        self.State = self._forwards(speed) # move forward 
                elif (direction == DIRECTION.CannotFind):
                    speed = 20
                    pauseTime = 0.3
                    self.State = self._turn(speed,ANTICLOCKWISE)
                elif (direction == DIRECTION.Left):
                    speed = 20
                    pauseTime =0.15
                    self.State = self._turn(speed,ANTICLOCKWISE)

                else:
                    self.State = self._turn(speed,CLOCKWISE)
                    speed = 20
                    pauseTime =0.15
                sleep(pauseTime)
                self.x_pos, self.y_pos, self.rot = self._updatePos()
            self._stop()
            print(f'Reached {self.x_pos}, {self.y_pos} with rot of {self.rot}\n')
        except KeyboardInterrupt:
            self._exemptExit()   

    def hitBallBetter(self)->None:
        oldDirection = None
        notHit = True   
        pauseTime = 0.2 
        try :
            while (noHit): # while not close enough to ball 
                (direction, temp, distance)= self.vision.detect() # run vision check 
                sleep(0.01)
                self.x_pos, self.y_pos, self.rot = self._updatePos()
                if (direction == DIRECTION.Ahead): # if ball ahead
                    speed = 50
                    pauseTime = 0.5
                    if (distance <0.35):
                        speed = 20
                        self.vision.tolerence = 100
                        self.State = self._forwards(speed)
                    if (distance <0.25): # if close to ball 
                        self.State = self._forwards(30)
                        self.x_pos, self.y_pos, self.rot = self._updatePos()
                        sleep(3.5)
                        self.x_pos, self.y_pos, self.rot = self._updatePos()
                        noHit = False # end 
                        sleep(0.1)
                        self._stop()
                    else: 
                        self.State = self._forwards(speed) # move forward 
                elif (direction == DIRECTION.CannotFind): #cannot find ball
                    speed = 20
                    pauseTime = 0.3
                    self.State = self._turn(speed,ANTICLOCKWISE)
                elif (direction == DIRECTION.Left):
                    if (oldDirection == DIRECTION.Right): # reduce oscillations 
                        speed -=5
                        speed = max(speed,15)
                    else:
                        speed = 20
                        pauseTime =0.15
                        self.State = self._turn(speed,ANTICLOCKWISE)

                else: #right 
                    if oldDirection ==DIRECTION.Left: # reduce oscillations
                        speed -=5
                        speed = max(speed,15)
                    else:
                        speed = 20
                        pauseTime =0.15
                        self.State = self._turn(speed,CLOCKWISE)
                        
                oldDirection = direction
                sleep(pauseTime)
                self.x_pos, self.y_pos, self.rot = self._updatePos()

            print(f'Reached {self.x_pos}, {self.y_pos} with rot of {self.rot}\n')
            self._stop()
            
        except KeyboardInterrupt:
            # STOP and RELEASE all pins 
           self._exemptExit()  
    
    #goto home
    def Home(self)->None:
        try:
            speed = 30 # define the speed at which the robot will move
            print(f' at X {self.x_pos}, Y {self.y_pos}, rot {self.rot}\n')

            # calculate the angle at which the robot needs to turn 
            angle = atan2(-self.y_pos,-self.x_pos)*180/pi
            angle = angle - self.rot # make it relative to the current position of the robot

            # calculate the distance that the robot needs to travel to return to home  
            distance = GLOBALSM1.wheelBaseCircumference*abs(angle)/360
            numPulses = (distance/GLOBALSM1.distancePerPulse)+self.EncoderL.encoderCount

            # rotate the amount calculated above
            if (angle >-1 and angle <1): # no rotation required 
                sleep(0.01)
            elif (angle>0):
                self.State = self._turn(speed,ANTICLOCKWISE) # rotate CCW
            else: 
                self.State = self._turn(speed,CLOCKWISE) # rotate CW 
            while (self.EncoderL.encoderCount <numPulses):
                x_pos, y_pos, rot = self._updatePos()
                sleep(0.02)
            self._stop()

            # calculate the forwards distance that needs to be travelled to arrive to starting positon
            distance = sqrt(self.x_pos**2 + self.y_pos**2)
            # update encoder count 
            encoderOldCount = self.EncoderL.encoderCount
            numPulses = (distance/GLOBALSM1.distancePerPulse)+encoderOldCount
            self.State = self.forwards(speed)

            while (self.EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
                self.x_pos, self.y_pos, self.rot = self.updatePos()
                sleep(0.02)
            self._stop
            GPIO.cleanup()

            # exit and release pins 
            self._stop
            GPIO.cleanup()
            
        except KeyboardInterrupt:
            # STOP and RELEASE all pins 
            self._exemptExit()




if __name__ == "__main__":
    robot = Sys5_Control() 
    # tell robot to do stuff between here 

    robot.hitBallBasic()
    robot.Home()
    
    #and here 
    robot.release() #release motor pins 



