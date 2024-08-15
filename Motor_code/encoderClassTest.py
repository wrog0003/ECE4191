from gpiozero import Button 
import RPi.GPIO as GPIO
import time
from math import pi, atan2, sqrt

#simple encoder class to track miltiple encoders on single system
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

motor1a = 17
motor1b = 27
motor2a = 23
motor2b = 24

# Setup the GPIO Pins to recieve the encoder pulses. 
motor1cha = 13
motor1chb = 19
motor2cha = 5
motor2chb = 6

#useful data units: meters
wheelDiameter = 0.054
wheelBase = 0.22
wheelBaseCircumference = pi*wheelBase
distancePerPulse = wheelDiameter*pi/(75*48)

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)

GPIO.setup(motor1a, GPIO.OUT) 
GPIO.setup(motor1b, GPIO.OUT)
GPIO.setup(motor2a, GPIO.OUT)
GPIO.setup(motor2b, GPIO.OUT)

pwm1a = GPIO.PWM(motor1a,1000)
pwm1b = GPIO.PWM(motor1b,1000)
pwm2a = GPIO.PWM(motor2a,1000)
pwm2b = GPIO.PWM(motor2b,1000)

def fowards(duty_cycle:float)->list[GPIO.PWM,GPIO.PWM]:
    # duty cycle between 0 - 100
        
    # drive the motor forwards 
    pwm1a.start(0)
    pwm1b.start(duty_cycle)
    pwm2a.start(0)
    pwm2b.start(duty_cycle)
    return [pwm1b,pwm2b]

def turn(duty_cycle:float,clockWise:bool)->list[GPIO.PWM,GPIO.PWM]:
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


#use this test to verify the direction for the encoder pins
def directionTest()->None:
    EncoderL = SimpleEncoder(motor1cha,motor1chb)
    EncoderR = SimpleEncoder(motor2cha,motor2chb)
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
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()
        EncoderR.end()

def pwmCalibration(speed:float):
    EncoderL = SimpleEncoder(motor1cha,motor1chb)
    try:
        fowards(speed)
        time.sleep(2)
        print(EncoderL.encoderCount)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
    except KeyboardInterrupt:
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()
#
def doughnuts(angle:float)->None:
    EncoderL = SimpleEncoder(motor1cha,motor1chb)
    EncoderR = SimpleEncoder(motor2cha,motor2chb)
    distance = angle*wheelBaseCircumference/360 #get the distance 
    pulses = distance/distancePerPulse
    turn(50,True)
    try:
        while (EncoderL.encoderCount<pulses):
            time.sleep(0.001)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
    except KeyboardInterrupt:
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()
        EncoderR.end()

##################################controller code 
def pwmControl(pwmDesired:float, pwmMessured:float, Kp:float,Ki:float,e_sum:float,pin:GPIO.PWM)->float:
    dutyCycle = min(max(0,pwmDesired+Kp*(pwmDesired-pwmMessured)+Ki*e_sum),100) # get the duty cycle 
    e_sum += (pwmDesired-pwmMessured)
    pin.start(dutyCycle)
    return e_sum

def gotTo(X:float,Y:float):
    Kp =1
    Ki =0
    angle = atan2(Y,X)*180/pi
    print(wheelBaseCircumference*pi)
    distance = wheelBaseCircumference*abs(angle)/360 # get the distance of the circle 
    print(distance)
    #by getting the circumference and then multiplying by the angle/360
    numPulses = distance/distancePerPulse
    EncoderL = SimpleEncoder(motor1cha,motor1chb)
    EncoderR = SimpleEncoder(motor2cha,motor2chb)
    speed =30 
    oldEncoderCountL = 0
    oldEncoderCountR = 0 
    errorRight = 0 
    leftPin = None
    rightPin = None 
    try: 
        # rotate
        if (angle >-1 and angle <1):
            time.sleep(0.01)
        elif (angle>0):
            [leftPin,rightPin]= turn(speed,False)
        else: 
            [leftPin,rightPin]=turn(speed,True)
        while (EncoderL.encoderCount <numPulses):
            newCount = EncoderR.encoderCount-oldEncoderCountR
            oldEncoderCountR = EncoderR.encoderCount
            errorRight= pwmControl(speed,newCount,Kp,Ki,errorRight,rightPin)
            time.sleep(0.02)
        # stop rotating 
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        # move 2 
        distance = sqrt(X**2+Y**2)
        encoderOldCount = EncoderL.encoderCount
        numPulses = (distance/distancePerPulse)+encoderOldCount # get the new final target pulses
        [leftPin,rightPin]=fowards(speed)
        while (EncoderL.encoderCount <numPulses):
            newCount = EncoderR.encoderCount-oldEncoderCountR
            oldEncoderCountR = EncoderR.encoderCount
            errorRight= pwmControl(speed,newCount,Kp,Ki,errorRight,rightPin)
            time.sleep(0.02)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        
    except KeyboardInterrupt:
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        GPIO.cleanup()
        EncoderL.end()
        EncoderR.end()

pwmCalibration(100)