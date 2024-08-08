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
counts_per_revolution = 48
distance_per_count = (math.pi * wheel_diameter) / counts_per_revolution

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCODER_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.BOTH, callback=update_left_encoder)
    GPIO.add_event_detect(ENCODER_LEFT_B, GPIO.BOTH, callback=update_left_encoder)
    GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.BOTH, callback=update_right_encoder)
    GPIO.add_event_detect(ENCODER_RIGHT_B, GPIO.BOTH, callback=update_right_encoder)

def update_left_encoder(channel):
    global left_count
    A = GPIO.input(ENCODER_LEFT_A)
    B = GPIO.input(ENCODER_LEFT_B)
    if A == B:
        left_count += 1
    else:
        left_count -= 1
    update_position()

def update_right_encoder(channel):
    global right_count
    A = GPIO.input(ENCODER_RIGHT_A)
    B = GPIO.input(ENCODER_RIGHT_B)
    if A == B:
        right_count += 1
    else:
        right_count -= 1
    update_position()

def update_position():
    global x, y, theta, left_count, right_count
    left_distance = left_count * distance_per_count
    right_distance = right_count * distance_per_count
    distance = (left_distance + right_distance) / 2
    delta_theta = (right_distance - left_distance) / wheel_base
    theta += delta_theta
    x += distance * math.cos(theta)
    y += distance * math.sin(theta)

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
