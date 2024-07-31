# ECE4191 31/07/2024 Emma Vladicic 
# Using Raspberry Pi Robotics Library to Drive motors forwards, backwards and reverse 

from gpiozero import Robot 
from time import sleep 

# assign Rpi GPIO pins for M1 (left motor)
left_forward = 27
left_back = 17

# assign Rpi GPIO pins for M2 (right motor)
right_forward = 23
right_back = 24

# Initialise the robot 
robot = Robot(left =(left_forward,left_back), right = (right_forward, right_back)) 

# decide on robot speed 
speed = 1

while True:
# Move the robot forwards 
    robot.forward(speed)

    sleep(0.5)

    # Move the robot backwards 
    robot.backward(speed)

    sleep(0.5)

    # Move the robot left 
    robot.left(speed)

    sleep(0.5)

    # Move the robot right 
    robot.right(speed)

    # Reverse the robot 
    robot.reverse(speed)