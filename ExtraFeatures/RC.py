from ..GLOBALSM1 import *
from ..SysB_MotorPos import SysB_MotorPos, CLOCKWISE, ANTICLOCKWISE

'''This code allows for wasd control of the robot'''

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM) # set pin types 

robot = SysB_MotorPos() # creates robot 

running = True 
speed = 30
print("Use the following commands to rc control the robot")
print("W = forward")
print("S = backward")
print("A = left")
print("D = right")
print("Q = exit")
print("E = set speed")
while running:
    action = input("Input action")
    if action.capitalize() == "W":
        robot.forwards(speed)
        
    elif action.capitalize() == "S":
        robot.backwards(speed)
    elif action.capitalize() == "A":
        robot.turn(speed, ANTICLOCKWISE)
    elif action.capitalize() == "D":
        robot.turn(speed, CLOCKWISE)
    elif action.capitalize() == "Q":
        running =False
    elif action.capitalize() == "E":
        targetSpeed = input("Enter speed between 15-70")
        try:
            speed = float(targetSpeed)
            if 14 < speed < 71:
                print("Please enter a number between 15-70")
                speed = 30
        except ValueError:
            print("Please enter a valid number")
    robot.delay(0.5)

del robot # exit properly 

