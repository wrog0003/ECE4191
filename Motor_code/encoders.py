import RPi.GPIO as GPIO
import time
import math

# Encoder pin definitions for left and right motors
ENCODER_LEFT_A = 17
ENCODER_LEFT_B = 18
ENCODER_RIGHT_A = 22
ENCODER_RIGHT_B = 23

# Global variables for position tracking
x = 0.0
y = 0.0
theta = 0.0

# Encoder counts and wheel specifications
left_count = 0
right_count = 0
wheel_diameter = 0.1  # meters
wheel_base = 0.3  # distance between wheels in meters
counts_per_revolution = 24

# Distance per count is the distance the robot travels per encoder count
# Calculated based on wheel's circumference and encoder resolution
distance_per_count = (math.pi * wheel_diameter) / counts_per_revolution

def setup():
    GPIO.setmode(GPIO.BCM)

    # Setting up GPIO
    GPIO.setup(ENCODER_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # If event is detected, call the respective functions
    GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.BOTH, callback=update_left_encoder)
    GPIO.add_event_detect(ENCODER_LEFT_B, GPIO.BOTH, callback=update_left_encoder)
    GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.BOTH, callback=update_right_encoder)
    GPIO.add_event_detect(ENCODER_RIGHT_B, GPIO.BOTH, callback=update_right_encoder)

def update_left_encoder(channel):
    #Declaring global variable
    global left_count
    # Check direction (backward or forward) based on the state of the B channel
    if GPIO.input(ENCODER_LEFT_A) == GPIO.input(ENCODER_LEFT_B):
        left_count += 1 #Moving forward
    else:
        left_count -= 1 #Moving backward
    update_position()

def update_right_encoder(channel):
    #Declaring global variable
    global right_count
    # Check direction (backward or forward) based on the state of the B channel
    if GPIO.input(ENCODER_RIGHT_A) == GPIO.input(ENCODER_RIGHT_B):
        right_count += 1 #Moving forward
    else:
        right_count -= 1 #Moving backward
    update_position()

def update_position():
    global x, y, theta, last_left_count, last_right_count
    #last_left_count and last_right_count stores the previous encoder counts
    #used to determine the number of new counts since the last update

    # Calculating the distance moved by each wheel since last update
    delta_left = (left_count - last_left_count) * distance_per_count
    delta_right = (right_count - last_right_count) * distance_per_count

    # Updating last_left_count and last_right_count
    last_left_count = left_count
    last_right_count = right_count

    delta_distance = (delta_left + delta_right) / 2 #average distance traveled in a straight line
    delta_theta = (delta_right - delta_left) / wheel_base #change of orientation of the robot

    # Update position
    theta += delta_theta #current robot orientation
    #(x,y) potion of the robot
    x += delta_distance * math.cos(theta)
    y += delta_distance * math.sin(theta)


def return_to_start():
    global x, y, theta
    target_x, target_y = 0.0, 0.0
    while abs(x - target_x) > 0.01 or abs(y - target_y) > 0.01:
        angle_to_target = math.atan2(target_y - y, target_x - x)
        angle_error = angle_to_target - theta
        distance_error = math.sqrt((target_x - x)**2 + (target_y - y)**2)
        
        # Adjust robot movement based on angle_error and distance_error
        # This part depends on the specific motor control setup

        # Example pseudo code for adjusting motors:
        # left_speed = distance_error - angle_error
        # right_speed = distance_error + angle_error
        # set_motor_speeds(left_speed, right_speed)
        
        time.sleep(0.1)  # Small delay to allow motor updates
    
    # Stop motors once the target is reached
    # stop_motors()

if __name__ == '__main__':
    try:
        setup()
        while True:
            # Main loop where the robot operates
            # For example, code to navigate to a ball and collect it
            
            # Once task is done, return to start
            return_to_start()
    except KeyboardInterrupt:
        GPIO.cleanup()
