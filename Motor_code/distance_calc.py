# ECE4191 6/08/2024 
# Using Raspberry Pi to recieve encoder data and calculate the distance travelle by a motor.

from gpiozero import Button 
import RPi.GPIO as GPIO
import time
##########################################
# Driving Motors Section
motor1a = 17
motor1b = 27
motor2a = 23
motor2b = 24

# Setup the GPIO Pins to recieve the encoder pulses. 
motor1cha = 13
motor1chb = 19
motor2cha = 16
motor2chb = 26

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

#################################################
# Parameters based on the system 
cpr = 48
wheel_diameter = 0.052
circumference = 3.14159 * wheel_diameter
gear_ratio = 75



# Sets the pin number selection to be not based on the phyiscal board pins 
#GPIO.setmode(GPIO.BCM)



# Setting up variables to capture encoder position
encoder_count = 0
last_A_state = 0
last_B_state = 0

# Callback function updates the encoder count based on the changes detected on the encoder pins
def encoder_callback(channel):
    global encoder_count, last_A_state, last_B_state

    # Read the current states of channel A and channel B
    current_A_state = motor1_cha.value
    current_B_state = motor1_chb.value
    #GPIO.input(motor1chb)
    # Determine the state transitions to calculate direction 
    # - If Channel A is leading then this is taken as clockwise. If Channel B is leading then this is anti-clockwise 
    #   Motor1 (Left Motor): Channel A leads B & Motor2 (Right Motor): Channel B leads A -> Back
    #   Motor1 (Left Motor): Channel B leads A & Motor2 (Right Motor): Channel B leads A -> Right 
    #   Motor1 (Left Motor): Channel A leads B & Motor2 (Right Motor): Channel A leads B -> Left 
    #   Motor1 (Left Motor): Channel B leads A & Motor2 (Right Motor): Channel A leads B -> Forward

    if current_A_state != last_A_state: # Channel A has changed
        if  current_A_state == current_B_state:
            encoder_count -= 1 # Reverse 
        else:
            encoder_count += 1 # Forward

    if current_B_state != last_B_state: # Channel B has changed
        if  current_A_state != current_B_state:    
            encoder_count -= 1 # Reverse 
        else:
            encoder_count += 1 # Forward


    # Update the last state
    last_A_state = current_A_state
    last_B_state = current_B_state


def calculate_distance(counts):
    # Calculate Revolutions 
    revolutions = counts/cpr 

    # calculate distance travelled
    distance = revolutions * circumference

    return distance


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


try:
    fowards(100)
    while True:
        distance_traveled = calculate_distance(encoder_count)

        # Print the Distance
        print(f"Distance: {distance_traveled:.2f}")

        time.sleep(0.1)  # Sleep to simulate a polling frequency that is twice the sampling frequency of the encoder signals to avoid aliasing. 

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