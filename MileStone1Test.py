#Written by Warren Rogan
import RPi.GPIO as GPIO
import time
from math import pi, atan2, sqrt, sin, cos

#from Motor_code.encoderClassTest import SimpleEncoder
from Sys4_Vision import Sys4_Vision
from ECE4191enums import DIRECTION
from Motor_code.encoderClass import SimpleEncoder


ANTICLOCKWISE = False
CLOCKWISE = True

# MOTOR OUTPUT PINS (DRIVING)

# Motor Left 
motor1a = 17
motor1b = 27

# Motor Right 
motor2a = 23
motor2b = 24

# MOTOR INPUT PINS (ENCODERS)

# Define the GPIO Pins to recieve the encoder pulses. 

# Motor Left 
motor1cha = 13
motor1chb = 19

# Motor Right 
motor2cha = 5
motor2chb = 6

# Useful data units: meters
wheelDiameter = 0.054 # diameter of the wheel
wheelBase = 0.22 # distance between the centre of both wheels 
wheelBaseCircumference = pi*wheelBase # circumference of the wheel 
distancePerPulse = wheelDiameter*pi/(75*48) # how far the robot can move per pulse of the encoders

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)

# Set up the output pins 
GPIO.setup(motor1a, GPIO.OUT) 
GPIO.setup(motor1b, GPIO.OUT)
GPIO.setup(motor2a, GPIO.OUT)
GPIO.setup(motor2b, GPIO.OUT)

# Set up the PWM pins
pwm1a = GPIO.PWM(motor1a,1000)
pwm1b = GPIO.PWM(motor1b,1000)
pwm2a = GPIO.PWM(motor2a,1000)
pwm2b = GPIO.PWM(motor2b,1000)


def fowards(duty_cycle:float)->list[GPIO.PWM,GPIO.PWM]:
   
    # input: duty cycle between 0 - 100
    # output: return the active pins

    # drive the motor forwards 
    pwm1a.start(0)
    pwm1b.start(duty_cycle)
    pwm2a.start(0)
    pwm2b.start(duty_cycle)

    return [pwm1b,pwm2b] 
def backwards(duty_cycle:float)->list[GPIO.PWM,GPIO.PWM]:
   
    # input: duty cycle between 0 - 100
    # output: return the active pins

    # drive the motor forwards 
    pwm1b.start(0)
    pwm1a.start(duty_cycle)
    pwm2b.start(0)
    pwm2a.start(duty_cycle)

    return [pwm1a,pwm2a] 

def turn(duty_cycle:float,clockWise:bool)->list[GPIO.PWM,GPIO.PWM]:

    # input: duty cycle between 0 - 100, if you want to turn clockwise or anticlockwise 
    # output: return the active pins

    if clockWise:
        pwm1a.start(0)
        pwm1b.start(duty_cycle)
        pwm2a.start(duty_cycle)
        pwm2b.start(0)
        return [pwm1b,pwm2a]
    else:
        pwm1a.start(duty_cycle)
        pwm1b.start(0)
        pwm2a.start(0)
        pwm2b.start(duty_cycle)
        return [pwm1a,pwm2b]

def stop()->None:
    pwm1a.stop()
    pwm1b.stop()
    pwm2a.stop()
    pwm2b.stop()

# simple test to make sure that the robot can turn towards the ball 
def turnAtBallTest():
    #init function variables 
    print("entered")
    speed = 20
    vision = Sys4_Vision()
    NotAhead = True # init ending variable 
    try :
        while (NotAhead): 
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction)
            if (direction == DIRECTION.Ahead): # if ball is ahead
                NotAhead = False # change to end while loop 
                pwm1a.stop()
                pwm1b.stop()
                pwm2a.stop()
                pwm2b.stop()
            elif (direction == DIRECTION.CannotFind): # if no ball detected in current frame 
                turn(speed, ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                turn(speed, ANTICLOCKWISE)
            else:
                turn(speed,CLOCKWISE) 
            time.sleep(0.2) # delay 200ms 
        # exit and release pins 
        
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

# simple test to get the robot to find the ball, turn to the ball and get close to the ball 
def hitBallTestBasic():
    # init function variables 
    speed = 50
    pauseTime = 0.2
    vision = Sys4_Vision()
    noHit = True # define stop condition 
    try :
        while (noHit): # while not close enough to ball 
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction.name)
            print(distance)
            if (direction == DIRECTION.Ahead): # if ball ahead
                speed = 100
                pauseTime = 0.5
                if (distance <0.4):
                    speed = 20
                    vision.tolerence = 100
                    fowards(speed)
                if (distance <0.3): # if close to ball 
                    fowards(30)
                    time.sleep(4)
                    noHit = False # end 
                    pwm1a.stop()
                    pwm1b.stop()
                    pwm2a.stop()
                    pwm2b.stop()
                else: 
                    fowards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind):
                speed = 20
                pauseTime = 0.3
                turn(speed,ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                speed = 20
                pauseTime =0.15
                turn(speed,ANTICLOCKWISE)

            else:
                turn(speed,CLOCKWISE)
                speed = 20
                pauseTime =0.15
            time.sleep(pauseTime)
            
        # exit and release pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

def hitBallTestBetter():
    # init function variables 
    speed = 50
    pauseTime = 0.2
    vision = Sys4_Vision()
    oldDirection = None 
    noHit = True # define stop condition 
    try :
        while (noHit): # while not close enough to ball 
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction.name)
            print(distance)
            if (direction == DIRECTION.Ahead): # if ball ahead
                speed = 100
                pauseTime = 0.5
                if (distance <0.35):
                    speed = 20
                    vision.tolerence = 100
                    fowards(speed)
                if (distance <0.25): # if close to ball 
                    fowards(30)
                    time.sleep(3.5)
                    noHit = False # end 
                    pwm1a.stop()
                    pwm1b.stop()
                    pwm2a.stop()
                    pwm2b.stop()
                else: 
                    fowards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind): #cannot find ball
                speed = 20
                pauseTime = 0.3
                turn(speed,ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                if (oldDirection == DIRECTION.Right): # reduce occilations 
                    speed -=5
                    speed = max(speed,10)
                else:
                    speed = 20
                    pauseTime =0.15
                    turn(speed,ANTICLOCKWISE)

            else: #right 
                if oldDirection ==DIRECTION.Left: # reduce occilations
                    speed -=5
                    speed = max(speed,10)
                else:
                    turn(speed,CLOCKWISE)
                    speed = 20
                    pauseTime =0.15
            oldDirection = direction
            time.sleep(pauseTime)
            
        # exit and release pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

#returns x, y, rot
def updatePos(encoderL:SimpleEncoder,encoderR:SimpleEncoder,x_old:float,y_old:float,rot_old:float)->list[float]:
    # get data
    [newL, dirL, oldL] = encoderL.getValues()
    [newR, dirR, oldR] = encoderR.getValues()

    #difference
    delL = newL-oldL
    delR = newR-oldR

    #get average travelled distance 
    distanceAvg = ((delL*distancePerPulse)+(delR*distancePerPulse))/2 
    #determine direction
    if (dirL==dirR): #turning
        #define non changing aspects
        x = x_old
        y = y_old
        delAngle = distanceAvg*360/wheelBaseCircumference #convert from distance to angle 
        if dirL: #left 
            rot = rot_old-delAngle
        else: #right 
            rot = rot_old+delAngle
        #deal with limiting angle domain 
        if rot > 180:
            rot -=360
        elif rot <-180:
            rot += 360

    else:
        rot = rot_old
        if (dirR): #forward
            y= y_old+distanceAvg*sin(rot*pi/180)
            x = x_old+distanceAvg*cos(rot*pi/180) 
        else: #backward
            y= y_old-distanceAvg*sin(rot*pi/180)
            x = x_old-distanceAvg*cos(rot*pi/180) 
    return x,y,rot
        


#warrens interpretation of this 
def got2andHome(X:float,Y:float):

    #init base variables
    oldEncoderCountL = 0
    oldEncoderCountR = 0 
    x_pos = 0 #meters
    y_pos = 0
    rot = 0 #degrees
    speed =50

    # get angle to point
    angle = atan2(Y,X)*180/pi
    distance = wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
    numPulses = distance/distancePerPulse
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor

    try:
        # rotate 
        if (angle >-1 and angle <1): # no rotation required 
            time.sleep(0.01)
        elif (angle>0):
            [leftPin,rightPin]= turn(speed,False)# rotate CCW
        else: 
            [leftPin,rightPin]=turn(speed,True)# rotatte CW 
        while (EncoderL.encoderCount <numPulses):
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            time.sleep(0.02)
        stop()

        distance = sqrt(X**2+Y**2) # calculate distance to drive forward 
        encoderOldCount = EncoderL.encoderCount # update encoder count 
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        fowards(speed) # drive forwards 
        while (EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            time.sleep(0.02)
        stop()
        #return code 
        #rotate 
        angle = atan2(-y_pos,-x_pos)*180/pi
        angle = angle -rot #make it relitive to current pos
        distance = wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
        numPulses = (distance/distancePerPulse)+EncoderL.encoderCount
        if (angle >-1 and angle <1): # no rotation required 
            time.sleep(0.01)
        elif (angle>0):
            [leftPin,rightPin]= turn(speed,ANTICLOCKWISE)# rotate CCW
        else: 
            [leftPin,rightPin]=turn(speed,CLOCKWISE)# rotatte CW 
        while (EncoderL.encoderCount <numPulses):
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            time.sleep(0.02)
        stop()

        #move 
        distance = sqrt(x_pos**2+y_pos**2) # calculate distance to drive forward 
        encoderOldCount = EncoderL.encoderCount # update encoder count 
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        fowards(speed) # drive forwards 
        while (EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            time.sleep(0.02)
        stop()
        GPIO.cleanup()

    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

def hitBallGetHome():
    # init function variables 
    speed = 50
    pauseTime = 0.2
    vision = Sys4_Vision()
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor
    oldEncoderCountL = 0
    oldEncoderCountR = 0 
    x_pos = 0 #meters
    y_pos = 0
    rot = 0 #degrees
    
    oldDirection = None 
    noHit = True # define stop condition 
    try :
        while (noHit): # while not close enough to ball 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            print(f'X {x_pos}, Y {y_pos}, rot {rot}\n')
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction.name)
            print(distance)
            if (direction == DIRECTION.Ahead): # if ball ahead
                speed = 100
                pauseTime = 0.5
                if (distance <0.35):
                    speed = 20
                    vision.tolerence = 100
                    fowards(speed)
                if (distance <0.25): # if close to ball 
                    fowards(30)
                    time.sleep(3.5)
                    x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
                    noHit = False # end 
                    pwm1a.stop()
                    pwm1b.stop()
                    pwm2a.stop()
                    pwm2b.stop()
                else: 
                    fowards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind): #cannot find ball
                speed = 20
                pauseTime = 0.3
                turn(speed,ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                if (oldDirection == DIRECTION.Right): # reduce occilations 
                    speed -=5
                    speed = max(speed,10)
                else:
                    speed = 20
                    pauseTime =0.15
                    turn(speed,ANTICLOCKWISE)

            else: #right 
                if oldDirection ==DIRECTION.Left: # reduce occilations
                    speed -=5
                    speed = max(speed,10)
                else:
                    turn(speed,CLOCKWISE)
                    speed = 20
                    pauseTime =0.15
            oldDirection = direction
            time.sleep(pauseTime)
            
        
        #return 2 home 
        print(f'X {x_pos}, Y {y_pos}, rot {rot}\n')
        angle = atan2(-y_pos,-x_pos)*180/pi
        angle = angle -rot #make it relitive to current pos
        distance = wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
        numPulses = (distance/distancePerPulse)+EncoderL.encoderCount
        if (angle >-1 and angle <1): # no rotation required 
            time.sleep(0.01)
        elif (angle>0):
            [leftPin,rightPin]= turn(speed,ANTICLOCKWISE)# rotate CCW
        else: 
            [leftPin,rightPin]=turn(speed,CLOCKWISE)# rotatte CW 
        while (EncoderL.encoderCount <numPulses):
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            time.sleep(0.02)
        stop()

        #move 
        distance = sqrt(x_pos**2+y_pos**2) # calculate distance to drive forward 
        encoderOldCount = EncoderL.encoderCount # update encoder count 
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        fowards(speed) # drive forwards 
        while (EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            time.sleep(0.02)
        stop()
        GPIO.cleanup()

        # exit and release pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

    
#got2andHome(0.5,0.2)
hitBallGetHome()
