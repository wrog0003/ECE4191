# ECE4191 6/08/2024 
# Using Raspberry Pi to recieve encoder data and calculate the distance travelle by the robot.

import RPi.GPIO as GPIO 
import time

# Parameters based on the system 
cpr = 48
wheel_diameter = 0.1
circumference = 3.14159 * wheel_diameter

# Setup the GPIO Pins to recieve the encoder pulses. 
motor1cha = 5
motor1chb = 6
motor2cha = 16
motor2chb = 26

# Sets the pin number selection to be not based on the phyiscal board pins 
GPIO.setmode(GPIO.BCM)

# Sets up each GPIO Pin as an Input with a pull up resistor which prevents the pins from picking up noise. 
GPIO.setup(motor1cha, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
GPIO.setup(motor1chb, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(motor2cha, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(motor2chb, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Setting up variables to capture encoder position
encoder_count = 0
last_A_state = 0
last_B_state = 0

# Callback function updates the encoder count based on the changes detected on the encoder pins
def encoder_callback(channel):
    global encoder_count, last_A_state, last_B_state

    # Read the current states of channel A and channel B
    current_A_state = GPIO.input(motor1cha)
    current_B_state = GPIO.input(motor1chb)

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
GPIO.add_event_detect(motor1cha, GPIO.BOTH, callback=encoder_callback)
GPIO.add_event_detect(motor1chb, GPIO.BOTH, callback=encoder_callback)




try:
    while True:
        distance_traveled = calculate_distance(encoder_count)

        # Print the Distance
        print(f"Distance: {distance_traveled:.2f}")

        time.sleep(0.01)  # Sleep to simulate a polling frequency that is twice the sampling frequency of the encoder signals to avoid aliasing. 

except KeyboardInterrupt:
    # Cleanup GPIO pins on exit
    GPIO.cleanup()
