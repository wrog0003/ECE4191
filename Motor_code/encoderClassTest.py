from gpiozero import Button 
import RPi.GPIO as GPIO
import time

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

def fowards(duty_cycle:float):
    # duty cycle between 0 - 100
        
    # drive the motor forwards 
    pwm1a.start(0)
    pwm1b.start(duty_cycle)
    pwm2a.start(0)
    pwm2b.start(duty_cycle)


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

    # EncoderL.end()
    # EncoderR.end()

def 
directionTest()