# ECE4191 31/07/2024 
# Using Raspberry Pi to Drive motors forwards, backwards and reverse 

import RPi.GPIO as GPIO 
from time import sleep 

# assign Rpi GPIO pins for M1 (left motor)
left_forward = 27
left_back = 17

# assign Rpi GPIO pins for M2 (right motor)
right_forward = 23
right_back = 24

# PI controller to achieve desired duty cycele
# Source: Unit GIT Page

#def pwm_control(w_desired,w_measured,Kp,Ki,e_sum):
    
#    duty_cycle = min(max(0,Kp*(w_desired-w_measured) + Ki*e_sum),1)
#    e_sum = e_sum + w_desired-w_measured
    
#    return duty_cycle, e_sum

GPIO.cleanup()
stop_cmd = 0
# Sets up GPIO Pins as outputs
# A: Backwards
# B: Forwards
motor1a = 17
motor1b = 27
motor2a = 23
motor2b = 24

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)

GPIO.setup(motor1a, GPIO.OUT) 
GPIO.setup(motor1b, GPIO.OUT)
GPIO.setup(motor2a, GPIO.OUT)
GPIO.setup(motor2b, GPIO.OUT) 

# set the GPIO pins as PWM 
pwm1a = GPIO.PWM(motor1a,1000)
pwm1b = GPIO.PWM(motor1b,1000)
pwm2a = GPIO.PWM(motor2a,1000)
pwm2b = GPIO.PWM(motor2b,1000)

def backwards(duty_cycle:float):
    #Duty cycle between 0-100

    while stop_cmd != 1:
        # Motor driving Backwards
        pwm1a.start(duty_cycle)
        pwm1b.start(0)
        pwm2a.start(duty_cycle)
        pwm2b.start(0)

    pwm1a.stop()
    pwm1b.stop()
    pwm2a.stop()
    pwm2b.stop()

        


def fowards(duty_cycle:float):
 
    # duty cycle between 0 - 100

    while stop_cmd !=1:
        
        # drive the motor forwards 
        pwm1a.start(0)
        pwm1b.start(duty_cycle)
        pwm2a.start(0)
        pwm2b.start(duty_cycle)

    pwm1a.stop()
    pwm1b.stop()
    pwm2a.stop()
    pwm2b.stop()


def left(duty_cycle:float):
    # duty cycle between 0 - 100

    while stop_cmd !=1:
        
        # drive the motor forwards 
        pwm1a.start(0)
        pwm1b.start(duty_cycle)
        pwm2a.start(duty_cycle)
        pwm2b.start(0)

    pwm1a.stop()
    pwm1b.stop()
    pwm2a.stop()
    pwm2b.stop()

def right(duty_cycle:float):
    # duty cycle between 0 - 100

    while stop_cmd != 1:

        # drive the left motor forward 
        pwm1a.start(duty_cycle)
        pwm1b.start(0)
        pwm2a.start(0)
        pwm2b.start(duty_cycle)

    pwm1a.stop()
    pwm1b.stop()
    pwm2a.stop()
    pwm2b.stop()



right(50)