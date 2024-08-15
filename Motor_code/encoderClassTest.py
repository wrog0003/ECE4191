from gpiozero import Button 
import RPi.GPIO as GPIO
import time
from math import pi, atan2, sqrt

# Set up a simple encoder class to track miltiple encoders on single system
class SimpleEncoder:
    #init to set up all sections 
    def __init__(self,Apin:int,Bpin:int) -> None:
        self.Apin = Button(Apin, pull_up=True) 
        self.Bpin = Button(Bpin, pull_up=True) 
        self.encoderCount = 0
        self.clockWise = False #facing in from outside
        self.Apin.when_pressed = self.encoderCallA
        self.Bpin.when_pressed = self.encoderCallB
        self.Apin.when_released = self.encoderCall
        self.Bpin.when_released = self.encoderCall

    # interrupt callback functions
    def encoderCallA(self,channel):
        self.encoderCount+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = True

    def encoderCallB(self,channel):
        self.encoderCount+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = False

    def encoderCall(self,channel):
        self.encoderCount+=1

    #get the state of the encoder 
    def getValues(self)->tuple[int,bool]:
        return [self.encoderCount,self.clockWise]
    
    #function call to relase pins
    def end(self)->None:
        self.Apin.close()
        self.Bpin.close() 

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


# Use this test to verify the direction for the encoder pins
def directionTest()->None:
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # define Motor 1 as a class
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # define Motor 2 as a class
    fowards(50)
    try:
        time.sleep(2)
        print("True is clockwise, false is anticlockwise")
        [count, direction] =EncoderL.getValues()
        print(f"Left: {direction}\n")
        [count, direction] =EncoderR.getValues()
        print(f"Right: {direction}\n")
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
        EncoderL.end()
        EncoderR.end()

def pwmCalibration(speed:float): # PWM Calibration code 

    # Inputs: speed to move at

    EncoderL = SimpleEncoder(motor1cha,motor1chb) # instantiate the left encoder 

    try:
        fowards(speed) # move forward at desired speed and print the number of counts
        time.sleep(2)
        print(EncoderL.encoderCount)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
    except KeyboardInterrupt:
        # STOP and RELEASE all GPIO pins
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()

def doughnuts(angle:float)->None:
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # Initalise Motor L 
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # Initialise Motor R 
    distance = angle*wheelBaseCircumference/360 # get the distance that you need to travel to reach that angle  
    pulses = distance/distancePerPulse # calculate the number of pulses that are required to complete the rotation
    turn(50,True) # call turn function, specify speed and whether you want to travel clockwise or anticlockwise 
    try:
        # while your encorder pulses are less than the desired pulses calcaulated for the turn, keep moving
        while (EncoderL.encoderCount<pulses): 
            time.sleep(0.001) 
        # once turn complete, release all resources
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
    except KeyboardInterrupt:
         # STOP and RELEASE all GPIO pins
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()
        EncoderR.end()

##################################controller code 
def pwmControl(pwmDesired:float, pwmMessured:float, Kp:float,Ki:float,e_sum:float,pin:GPIO.PWM)->float:
    # Inputs: 
    # pwmDesired: desired speed expressed as a PWM duty cycle
    # pwmMeasured: what the measured PWM of the motor actually is 
    # Kp: proptional constant 
    # Ki: integral constant 
    # e_sum: accumulation of errors 

    # Outputs 
    # e_sum = accumulation of errors 


    dutyCycle = min(max(0,Kp*(pwmDesired-pwmMessured)+Ki*e_sum),100) # calcualte the duty cycle to minimise error 
    e_sum += (pwmDesired-pwmMessured) # calculate the new accumualted error 
    pin.start(dutyCycle) # drive one motor at the new calculated speed 
    return e_sum


def gotTo(X:float,Y:float):
    # Function: takes in coordinates (X,Y) and moves to those co-ordinates 

    # Inputs: 
    # X: x coordinate (forwards)
    # Y: y coordinate (left)
    # this is the coordinate system we have defined 

    # calculate important information
    Kp =1
    Ki =0
    angle = atan2(Y,X)*180/pi
    print(wheelBaseCircumference*pi)

    distance = wheelBaseCircumference*abs(angle)/360 # get the distance that needs to be travelled to 
    # achieve required turn
    
    print(distance)
    #by getting the circumference and then multiplying by the angle/360

    numPulses = distance/distancePerPulse # get the number of pulses required to achieve the turn 
    EncoderL = SimpleEncoder(motor1cha,motor1chb) # set up Left Motor
    EncoderR = SimpleEncoder(motor2cha,motor2chb) # set up right Motor 
    speed =30 # define the speed at which to travel
    # set the encoder count to 0
    oldEncoderCountL = 0
    oldEncoderCountR = 0 
    errorRight = 0 
    errorLeft =0
    leftPin = None
    rightPin = None 
    try: 
        # rotate
        if (angle >-1 and angle <1): # no rotation required 
            time.sleep(0.01)
        elif (angle>0):
            [leftPin,rightPin]= turn(speed,False)# rotate CCW
        else: 
            [leftPin,rightPin]=turn(speed,True)# rotatte CW 
        while (EncoderL.encoderCount <numPulses):
            newCount = EncoderR.encoderCount-oldEncoderCountR
            oldEncoderCountR = EncoderR.encoderCount
            errorRight= pwmControl(speed,newCount,Kp,Ki,errorRight,rightPin)
            newCount = EncoderL.encoderCount-oldEncoderCountL
            oldEncoderCountL = EncoderL.encoderCount
            errorLeft= pwmControl(speed,newCount,Kp,Ki,errorLeft,leftPin)
            time.sleep(0.02)
        # stop rotating 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        
        # Now that the robot have reached its desired angle, we want to drive it forward a certain distance 
        distance = sqrt(X**2+Y**2) # calculate distance to drive forward 
        encoderOldCount = EncoderL.encoderCount # update encoder count 
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        fowards(speed) # drive forwards 
        while (EncoderL.encoderCount <numPulses):# keep going fowards until you reach the desired number of pulses 
            newCount = EncoderR.encoderCount-oldEncoderCountR
            oldEncoderCountR = EncoderR.encoderCount
            errorRight= pwmControl(speed,newCount,Kp,Ki,errorRight,rightPin)
            newCount = EncoderL.encoderCount-oldEncoderCountL
            oldEncoderCountL = EncoderL.encoderCount
            errorLeft= pwmControl(speed,newCount,Kp,Ki,errorLeft,leftPin)
            time.sleep(0.02)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        
    except KeyboardInterrupt:
         # STOP and RELEASE all GPIO pins
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()
        EncoderR.end()

gotTo(-1.1,-0.1)