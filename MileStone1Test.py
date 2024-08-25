#Written by Warren Rogan
import RPi.GPIO as GPIO
import time
from math import pi, atan2, sqrt, sin, cos

#from Motor_code.encoderClassTest import SimpleEncoder
from Sys4_Vision import Sys4_Vision
from ECE4191enums import DIRECTION, ACTION
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
distancePerPulse = wheelDiameter*pi/(74.8*24) # how far the robot can move per pulse of the encoders
duty_cycle_bias = 0.95

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


def forwards(duty_cycle:float)->ACTION:
   
    # input: duty cycle between 0 - 100
    # output: return the state of the robot

    # drive the motor forwards 
    pwm1a.start(0)
    pwm1b.start(duty_cycle)
    pwm2a.start(0)
    pwm2b.start(max(duty_cycle*duty_cycle_bias,5))

    return ACTION.FORWARD

def backwards(duty_cycle:float)->ACTION:
   
    # input: duty cycle between 0 - 100
    # output: return the state of the robot

    # drive the motor forwards 
    pwm1b.start(0)
    pwm1a.start(duty_cycle)
    pwm2b.start(0)
    pwm2a.start(max(duty_cycle*duty_cycle_bias,5))

    return ACTION.BACKWARD 

def turn(duty_cycle:float,clockWise:bool)->ACTION:

    # input: duty cycle between 0 - 100, if you want to turn clockwise or anticlockwise 
    # output: return the state of the robot

    if clockWise:
        pwm1a.start(0)
        pwm1b.start(duty_cycle)
        pwm2a.start(max(duty_cycle*duty_cycle_bias,5))
        pwm2b.start(0)
        return ACTION.RIGHT
    else:
        pwm1a.start(duty_cycle)
        pwm1b.start(0)
        pwm2a.start(0)
        pwm2b.start(max(duty_cycle*duty_cycle_bias,5))
        return ACTION.LEFT

def stop()->None:
    pwm1a.stop()
    pwm1b.stop()
    pwm2a.stop()
    pwm2b.stop()

#run basic tests to ensure that all gpio are in the correct positions
def calibrationTest()->None:
    try:
        forwards(30)
        time.sleep(1)
        backwards(30)
        time.sleep(1)
        turn(30,ANTICLOCKWISE)
        time.sleep(1)
        turn(30,CLOCKWISE)
        time.sleep(1)
        stop() 
        print("Test run")
        GPIO.cleanup()
    # clear up if exited manually
    except KeyboardInterrupt:
        stop()
        GPIO.cleanup()



# simple test to make sure that the robot can turn towards the ball 
def turnAtBallTest():
    #init function variables 
    speed = 20
    vision = Sys4_Vision()
    NotAhead = True # init ending variable 
    State =None 
    try :
        while (NotAhead): 
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction)
            if (direction == DIRECTION.Ahead): # if ball is ahead
                NotAhead = False # change to end while loop 
                stop()
            elif (direction == DIRECTION.CannotFind): # if no ball detected in current frame 
                State = turn(speed, ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                State = turn(speed, ANTICLOCKWISE)
            else:
                State = turn(speed,CLOCKWISE) 
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
    State = None
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
                    State = forwards(speed)
                if (distance <0.3): # if close to ball 
                    State = forwards(30)
                    time.sleep(4)
                    noHit = False # end 
                    stop()
                else: 
                    State = forwards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind):
                speed = 20
                pauseTime = 0.3
                State = turn(speed,ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                speed = 20
                pauseTime =0.15
                State = turn(speed,ANTICLOCKWISE)

            else:
                State = turn(speed,CLOCKWISE)
                speed = 20
                pauseTime =0.15
            time.sleep(pauseTime)
            
        # exit and release pins 
        stop()
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        stop()
        GPIO.cleanup()

def hitBallTestBetter():
    # init function variables 
    speed = 50
    pauseTime = 0.2
    vision = Sys4_Vision()
    oldDirection = None 
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor
    rot = 0
    x_pos = 0
    y_pos = 0
    noHit = True # define stop condition 
    State = None 
    try :
        while (noHit): # while not close enough to ball 
            (direction, temp, distance)= vision.detect() # run vision check 
            time.sleep(0.01)
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
            print(f'X {x_pos}, Y {y_pos}, rot {rot}\n')
            print(direction.name)
            print(distance)
            if (direction == DIRECTION.Ahead): # if ball ahead
                speed = 50
                pauseTime = 0.5
                if (distance <0.35):
                    speed = 20
                    vision.tolerence = 100
                    State = forwards(speed)
                if (distance <0.25): # if close to ball 
                    State = forwards(30)
                    print(f'Inintal X {x_pos},Y {y_pos}, rot {rot}')
                    x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
                    time.sleep(3.5)
                    x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
                    print(f'Final X {x_pos},Y {y_pos}, rot {rot}')
                    noHit = False # end 
                    time.sleep(0.1)
                    stop()
                else: 
                    State = forwards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind): #cannot find ball
                speed = 20
                pauseTime = 0.3
                State = turn(speed,ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                if (oldDirection == DIRECTION.Right): # reduce occilations 
                    speed -=5
                    speed = max(speed,15)
                else:
                    speed = 20
                    pauseTime =0.15
                    State = turn(speed,ANTICLOCKWISE)

            else: #right 
                if oldDirection ==DIRECTION.Left: # reduce occilations
                    speed -=5
                    speed = max(speed,15)
                else:
                    speed = 20
                    pauseTime =0.15
                    State = turn(speed,CLOCKWISE)
                    
            oldDirection = direction
            time.sleep(pauseTime)
            
            print(f'X {x_pos},Y {y_pos}, rot {rot}')
            #time.sleep(0.01)

        print(f'END X {x_pos},Y {y_pos}, rot {rot}')
        # exit and release pins 
        stop()
        GPIO.cleanup()
            
    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        stop()
        GPIO.cleanup()

#returns x, y, rot
def updatePos(encoderL:SimpleEncoder,encoderR:SimpleEncoder,x_old:float,y_old:float,rot_old:float,State:ACTION)->list[float]:
    # get data
    [newL, dirL, oldL] = encoderL.getValues()
    [newR, dirR, oldR] = encoderR.getValues()

    #difference
    delL = abs(newL-oldL)
    delR = abs(newR-oldR)
    #print(delL-delR)
    #get average travelled distance 
    distanceAvg = ((delL*distancePerPulse)+(delR*distancePerPulse))/2 
    x = 0
    y = 0 
    rot = 0 
    #determine direction
    if State == ACTION.FORWARD:
        rot = rot_old
        y= y_old+(distanceAvg*sin(rot*pi/180))
        x = x_old+(distanceAvg*cos(rot*pi/180))
    elif State ==ACTION.BACKWARD:
        rot = rot_old
        y= y_old-(distanceAvg*sin(rot*pi/180))
        x = x_old-(distanceAvg*cos(rot*pi/180))
    elif State == ACTION.LEFT:
        x= x_old
        y = y_old
        delAngle = distanceAvg*360/wheelBaseCircumference
        rot = rot_old+delAngle
    elif State == ACTION.RIGHT:
        x= x_old
        y = y_old 
        delAngle = distanceAvg*360/wheelBaseCircumference
        rot = rot_old-delAngle
    '''
    if (dirL==dirR): #turning
        #define non changing aspects
        x = x_old
        y = y_old
        delAngle = distanceAvg*360/wheelBaseCircumference #convert from distance to angle 
        if dirL: 
            print("left")
            rot = rot_old+delAngle
        else: #
            print("right")
            rot = rot_old-delAngle
        #deal with limiting angle domain 
        if rot > 180:
            rot -=360
        elif rot <-180:
            rot += 360

    else:
        
        rot = rot_old
        if (dirR): #backwards
            y= y_old+(distanceAvg*sin(rot*pi/180))
            x = x_old+(distanceAvg*cos(rot*pi/180) )
            print("forwards")
        else: #forwards

            y= y_old-(distanceAvg*sin(rot*pi/180))
            x = x_old-(distanceAvg*cos(rot*pi/180) )
            print("backwards")
    '''
    return x,y,rot
        
#warrens interpretation of this 
def got2andHome(X:float,Y:float):

    #init base variables
    oldEncoderCountL = 0
    oldEncoderCountR = 0 
    x_pos = 0 #meters
    y_pos = 0
    rot = 0 #degrees
    speed =30

    # get angle to point
    angle = atan2(Y,X)*180/pi
    distance = wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
    numPulses = distance/distancePerPulse
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor
    State = None 

    try:
        # rotate 
        if (angle >-1 and angle <1): # no rotation required 
            time.sleep(0.01)
        elif (angle>0):
            State= turn(speed,False)# rotate CCW
        else: 
            State=turn(speed,True)# rotatte CW 
        while (EncoderL.encoderCount <numPulses):
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
            time.sleep(0.02)
        stop()

        distance = sqrt(X**2+Y**2) # calculate distance to drive forward 
        encoderOldCount = EncoderL.encoderCount # update encoder count 
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        State = forwards(speed) # drive forwards 
        while (EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
            time.sleep(0.02)
        stop()

        
        #return code 
        print(f'Point: X{x_pos}, Y{y_pos}, rot{rot}')
        #rotate 
        angle = atan2(-y_pos,-x_pos)*180/pi
        angle = angle -rot #make it relitive to current pos
        distance = wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
        numPulses = (distance/distancePerPulse)+EncoderL.encoderCount
        if (angle >-1 and angle <1): # no rotation required 
            time.sleep(0.01)
        elif (angle>0):
            State= turn(speed,ANTICLOCKWISE)# rotate CCW
        else: 
            State=turn(speed,CLOCKWISE)# rotatte CW 
        while (EncoderL.encoderCount <numPulses):
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
            time.sleep(0.02)
        stop()

        #move 
        distance = sqrt(x_pos**2+y_pos**2) # calculate distance to drive forward 
        encoderOldCount = EncoderL.encoderCount # update encoder count 
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        forwards(speed) # drive forwards 
        while (EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
            time.sleep(0.02)
        stop()
        GPIO.cleanup()

    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        stop()
        GPIO.cleanup()

def hitBallGetHome():
    # init function variables 
    speed = 20
    pauseTime = 0.2
    vision = Sys4_Vision()
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor
    x_pos = 0 #meters
    y_pos = 0
    rot = 0 #degrees
    
    oldDirection = None 
    noHit = True # define stop condition 
    State = None 
    try :
        while (noHit): # while not close enough to ball 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
            print(f'X {x_pos}, Y {y_pos}, rot {rot}\n')
            (direction, temp, distance)= vision.detect() # run vision check 
            print(direction.name)
            print(distance)
            if (direction == DIRECTION.Ahead): # if ball ahead
                speed = 30
                pauseTime = 0.6
                if (distance <0.35):
                    speed = 20
                    vision.tolerence = 100
                    State = forwards(speed)
                if (distance <0.25): # if close to ball 
                    State = forwards(30)
                    print(f'Inintal X {x_pos},Y {y_pos}, rot {rot}')
                    time.sleep(3.5)
                    x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
                    print(f'Final X {x_pos},Y {y_pos}, rot {rot}')
                    noHit = False # end 
                    stop()
                else: 
                    State = forwards(speed) # move forward 
            elif (direction == DIRECTION.CannotFind): #cannot find ball
                speed = 10
                pauseTime = 0.3
                State = turn(speed,ANTICLOCKWISE)
            elif (direction == DIRECTION.Left):
                if (oldDirection == DIRECTION.Right): # reduce occilations 
                    speed -=5
                    speed = max(speed,15)
                else:
                    speed = 10
                    pauseTime =0.15
                    State =turn(speed,ANTICLOCKWISE)

            else: #right 
                if oldDirection ==DIRECTION.Left: # reduce occilations
                    speed -=5
                    speed = max(speed,15)
                else:
                    speed = 10
                    pauseTime =0.15
                    State =turn(speed,CLOCKWISE)
                    
            oldDirection = direction
            time.sleep(pauseTime)

        time.sleep(1)
        # go back to disengauge from the ball
        State = backwards(50) 
        time.sleep(1)
        x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
        State = forwards(30) 
        time.sleep(0.5)
        x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
        
        #return 2 home 
        speed = 30
        print(f' at X {x_pos}, Y {y_pos}, rot {rot}\n')
        angle = atan2(-y_pos,-x_pos)*180/pi
        angle = angle -rot #make it relitive to current pos
        distance = wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
        numPulses = (distance/distancePerPulse)+EncoderL.encoderCount
        if (angle >-1 and angle <1): # no rotation required 
            time.sleep(0.01)
        elif (angle>0):
            State= turn(speed,ANTICLOCKWISE)# rotate CCW
        else: 
            State=turn(speed,CLOCKWISE)# rotatte CW 
        while (EncoderL.encoderCount <numPulses):
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
            time.sleep(0.02)
        stop()

        #move 
        distance = sqrt(x_pos**2+y_pos**2) # calculate distance to drive forward 
        encoderOldCount = EncoderL.encoderCount # update encoder count 
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        State =forwards(speed) # drive forwards 
        while (EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot,State)
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

    
def turnForAWhile():
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor
    rot = 0
    x_pos =0
    y_pos = 0
    try:
        while True:
            turn(50,CLOCKWISE)
            time.sleep(0.2)
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
            print(rot)
            

    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        print(rot)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()

def calibrateDegrees(angle:float):
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor
    rot = 0
    x_pos =0
    y_pos = 0
    try:
        while (abs(rot-angle)>1):
            turn(50,CLOCKWISE)
            time.sleep(0.02)
            x_pos, y_pos, rot = updatePos(EncoderL,EncoderR, x_pos,y_pos,rot)
        GPIO.cleanup()
            

    except KeyboardInterrupt:
        # STOP and RELEASE all pins 
        print(rot)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()
        EncoderR.end()
#got2andHome(0.5,0.2)
# add 10ms delay between camera and location
hitBallGetHome()