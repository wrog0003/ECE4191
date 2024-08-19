from gpiozero import Motor
from gpiozero import Button
from signal import pause
import math
import time

# Constants
WHEEL_DIAMETER = 0.1  # Wheel diameter in meters
WHEEL_BASE = 0.3      # Distance between wheels in meters
COUNTS_PER_REV = 48  # Full quadrature counts per revolution
DIST_PER_COUNT = (math.pi * WHEEL_DIAMETER) / COUNTS_PER_REV

# GPIO pins for encoders
LEFT_ENCODER_A_PIN = 17
LEFT_ENCODER_B_PIN = 27
RIGHT_ENCODER_A_PIN = 18
RIGHT_ENCODER_B_PIN = 22

# Position and orientation
x = 0.0
y = 0.0
theta = 0.0

left_count = 0
right_count = 0
last_left_count = 0
last_right_count = 0

# Set up the motors (assuming motor controller is wired to these GPIO pins)
left_motor = Motor(forward=4, backward=14)
right_motor = Motor(forward=17, backward=18)

# Set up the encoder pins as buttons for detecting changes
left_encoder_a = Button(LEFT_ENCODER_A_PIN)
right_encoder_a = Button(RIGHT_ENCODER_A_PIN)

def left_encoder_change():
    global left_count
    # Check direction using the B channel
    # Incrementing for forward and decrementing for backwards
    if gpiozero.Device.pin_factory.pin(LEFT_ENCODER_B_PIN).state == gpiozero.Device.pin_factory.pin(LEFT_ENCODER_A_PIN).state:
        left_count += 1
    else:
        left_count -= 1

def right_encoder_change():
    global right_count
    # Check direction using the B channel
    # Incrementing for forward and decrementing for backwards
    if gpiozero.Device.pin_factory.pin(RIGHT_ENCODER_B_PIN).state == gpiozero.Device.pin_factory.pin(RIGHT_ENCODER_A_PIN).state:
        right_count += 1
    else:
        right_count -= 1

def update_position():
    global x, y, theta, last_left_count, last_right_count

    # Calculating thhe distance moved by each wheel
    # by getting the difference between the current and previoous count
    # multiplied it by the distance per count
    delta_left = (left_count - last_left_count) * DIST_PER_COUNT
    delta_right = (right_count - last_right_count) * DIST_PER_COUNT

    # Storing previous count for the next cycle
    last_left_count = left_count
    last_right_count = right_count

    # Calculating change in distance and angle
    # Distance is the average of the movement to the right and left
    # Angle is the difference of the movement to the right and left divided by the wheel base
    delta_distance = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / WHEEL_BASE

    # Update position using trigonometric identity
    theta += delta_theta
    x += delta_distance * math.cos(theta)
    y += delta_distance * math.sin(theta)

# Attach the encoder callbacks to the encoder signal changes
left_encoder_a.when_pressed = left_encoder_change
right_encoder_a.when_pressed = right_encoder_change

# Example loop to update the position periodically
try:
    while True:
        update_position()
        print(f"Position: (x={x:.2f}, y={y:.2f}, theta={theta:.2f})")
        time.sleep(0.1)

finally:
    left_motor.stop()
    right_motor.stop()