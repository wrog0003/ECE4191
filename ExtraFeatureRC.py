
from SysB_MotorPos import SysB_MotorPos, CLOCKWISE, ANTICLOCKWISE
from SysC_BallCollection import SysC_BallCollection

'''This code allows for wasd control of the robot'''

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM) # set pin types 

robot = SysB_MotorPos() # creates robot 
conveyer = SysC_BallCollection()


running = True 
speed = 30
print("Use the following commands to rc control the robot")
print("W = forward  S = backward")
print("A = left     D = right")
print("Q = exit     E = set speed")
print("L = load     U=unload")
while running:
    action = input()
    if action.capitalize() == "W":
        robot.forwards(speed)
        robot.delay(0.5)
    elif action.capitalize() == "S":
        robot.backwards(speed)
        robot.delay(0.5)
    elif action.capitalize() == "A":
        robot.turn(speed, ANTICLOCKWISE)
        robot.delay(0.5)
    elif action.capitalize() == "D":
        robot.turn(speed, CLOCKWISE)
        robot.delay(0.5)
    elif action.capitalize() == "L":
        conveyer.addBallToSystem()
        conveyer.ballCount +=1
    elif action.capitalize() == "U":
        conveyer.unloadBalls()
    elif action.capitalize() == "Q":
        running =False
    elif action.capitalize() == "E":
        targetSpeed = input("Enter speed between 15-70")
        try:
            speed = float(targetSpeed)
            if not (14 < speed < 71):
                print("Please enter a number between 15-70")
                speed = 30
        except ValueError:
            print("Please enter a valid number")
    else:
        print("Not a recongnised command")
    robot.stop()

del robot # exit properly 


