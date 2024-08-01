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


# Sets up GPIO Pins as outputs
# A: Froward
# B: Backward
motor1a = 17
motor1b = 27
motor2a = 23
motor2b = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1a, GPIO.OUT) 
GPIO.setup(motor1b, GPIO.OUT)
GPIO.setup(motor2a, GPIO.OUT)
GPIO.setup(motor2b, GPIO.OUT) 


pwm1a = GPIO.PWM(motor1a,1000)
pwm1b = GPIO.PWM(motor1b,1000)

# Motor driving forward
pwm1a.start(50)
pwm1b.start(0)
pwm2a.start(50)
pwm2b.start(0)

sleep(10)

pwm1a.stop()
pwm1b.stop()
GPIO.cleanup()
    
