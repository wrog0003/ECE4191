#Oringal written by Josh 
#Modifed by Warren Rogan and Emma Vladicic 

from gpiozero import Button 
import RPi.GPIO as GPIO
import time
from math import pi

#simple encoder class to track miltiple encoders on single system
class SimpleEncoder:
    #init to set up all sections 
    def __init__(self,Apin:int,Bpin:int) -> None:
        self.Apin = Button(Apin, pull_up=True) 
        self.Bpin = Button(Bpin, pull_up=True) 
        self.encoderCount = 0
        self.clockWise = False
        self.Apin.when_pressed = self.encoderCallA
        self.Bpin.when_pressed = self.encoderCallB
        self.Apin.when_released = self.encoderCall
        self.Bpin.when_released = self.encoderCall

    # interrupt callback functions
    def encoderCallA(self,channel):
        encoder_count+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = False

    def encoderCallB(self,channel):
        encoder_count+=1 # increment encoder count
        if (self.Apin.value and self.Bpin.value):
            self.clockWise = True

    def encoderCall(self,channel):
        encoder_count+=1

    #get the state of the encoder 
    def getValues(self)->tuple[int,bool]:
        return [self.encoderCount,self.clockWise]
    
    #function call to relase pins
    def end(self)->None:
        self.Apin.close()
        self.Bpin.close() 





##### Useful variables
wheelDiamter = 0.054 #meters


##########################################
# Driving Motors Section
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

# Sets up each GPIO Pin as an Input with a pull up resistor which prevents the pins from picking up noise. 
motor1_cha = Button(motor1cha, pull_up=True) 
motor1_chb = Button(motor1chb, pull_up=True) 
#GPIO.setup(motor2cha, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(motor2chb, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# set the GPIO pins as PWM 
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


# Sets the pin number selection to be not based on the phyiscal board pins 
#GPIO.setmode(GPIO.BCM)



# Setting up variables to capture encoder position
encoder_count = 0
last_A_state = 0
last_B_state = 0

# Callback function updates the encoder count based on the changes detected on the encoder pins
def encoder_callback(channel):
    global encoder_count
    encoder_count +=1





# Event detection for encoder pins 
# - GPIO.Both indicates that this callback function is called whenever there is a 
#   rising or falling edge on either or the motor GPIO Pins. 
# - Calls the encoder_callback function when the event occurs. 
motor1_cha.when_pressed = encoder_callback
motor1_chb.when_pressed = encoder_callback
motor1_cha.when_released = encoder_callback
motor1_chb.when_released = encoder_callback
#GPIO.add_event_detect(motor1cha, GPIO.RISING, callback=encoder_callback)
#GPIO.add_event_detect(motor1chb, GPIO.RISING, callback=encoder_callback)

def singleRevTest()->None:
    singleRev = 74*48 
    try:
        fowards(25)
        while (encoder_count<singleRev):
            time.sleep(0.001)
        print(encoder_count)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        # Cleanup GPIO pins on exit
        GPIO.cleanup()
        motor1_cha.close()
        motor1_chb.close()
    except KeyboardInterrupt:
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        # Cleanup GPIO pins on exit
        GPIO.cleanup()
        motor1_cha.close()
        motor1_chb.close()
        print("GPIO cleanup done. Exiting gracefully.")

def distanceTest(length:float)->None:
    disPerPulse = pi*wheelDiamter/(75*48)
    try:
        fowards(100)
        while(disPerPulse*encoder_count<length):
            time.sleep(0.001)
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        # Cleanup GPIO pins on exit
        GPIO.cleanup()
        motor1_cha.close()
        motor1_chb.close()
    except KeyboardInterrupt:
        pwm1a.stop()
        pwm1b.stop()
        pwm2a.stop()
        pwm2b.stop()
        # Cleanup GPIO pins on exit
        GPIO.cleanup()
        motor1_cha.close()
        motor1_chb.close()
        print("GPIO cleanup done. Exiting gracefully.")

distanceTest(0.5)