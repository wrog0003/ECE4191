# ECE4191 31/07/2024 
# Using Raspberry Pi to Drive motors forwards, backwards and reverse 

import RPi.GPIO as GPIO 
from time import sleep 

# In1 - 27
# In2 - 17
# In3 - 23
# In4 - 24

# PI controller to achieve desired duty cycele
# Source: Unit GIT Page

#def pwm_control(w_desired,w_measured,Kp,Ki,e_sum):
    
#    duty_cycle = min(max(0,Kp*(w_desired-w_measured) + Ki*e_sum),1)
#    e_sum = e_sum + w_desired-w_measured
    
#    return duty_cycle, e_sum

#GPIO.cleanup()
#stop_cmd = 0

# Motor one: In1 and In2 which drives the right motor 
# Motor two: In3 and In4 which drives the left motor

# Sets up GPIO Pins as outputs
# A: Backwards
# B: Forwards
# Motor 1: Left Motor 
# Motor 2: Right Motor 
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

    # Motor driving Backwards
    pwm1a.start(duty_cycle)
    pwm1b.start(0)
    pwm2a.start(duty_cycle)
    pwm2b.start(0)

        
def fowards(duty_cycle:float):
    # duty cycle between 0 - 100
        
    # drive the motor forwards 
    pwm1a.start(0)
    pwm1b.start(duty_cycle)
    pwm2a.start(0)
    pwm2b.start(duty_cycle)


def left(duty_cycle:float):
    # duty cycle between 0 - 100

    # drive the motor forwards 
    pwm1a.start(0)
    pwm1b.start(duty_cycle)
    pwm2a.start(duty_cycle)
    pwm2b.start(0)


def right(duty_cycle:float):
    # duty cycle between 0 - 100

    # drive the left motor forward 
    pwm1a.start(duty_cycle)
    pwm1b.start(0)
    pwm2a.start(0)
    pwm2b.start(duty_cycle)



try:
    while True:
        fowards(25) 

except KeyboardInterrupt:
    pwm1a.stop()
    pwm1b.stop()
    pwm2a.stop()
    pwm2b.stop()
    # Cleanup GPIO settings before exiting
    GPIO.cleanup()
    print("GPIO cleanup done. Exiting gracefully.")

