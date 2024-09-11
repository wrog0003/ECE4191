import cv2
import numpy as np
#import RPi.GPIO as GPIO
#from gpiozero import Button
from ECE4191enums import STATE, DIRECTION, ACTION  # Importing Enums
import time



# GPIO Setup for limit switch
LIMIT_SWITCH_PIN = 17  # Adjust according to your setup
#limit_switch = Button(LIMIT_SWITCH_PIN)

# Setup camera
cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)  # Assuming you're using the default camera

# Known width of the box in real world (in cm)
KNOWN_WIDTH = 30.0  # Adjust to the actual width of the box

# Focal length (can be calculated based on some sample images)
FOCAL_LENGTH = 615  # Adjust based on your camera specs

# Current state of the robot
robot_state = STATE.null
current_direction = DIRECTION.CannotFind
current_action = ACTION.FORWARD

def find_box(frame):
    """ Detect the cardboard box in the frame """
    global current_direction
    # Convert to HSV color space to detect the brown color of the box
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the brown color range
    lower_brown = np.array([15, 40, 100])  # Lower bound in HSV
    upper_brown = np.array([50, 180, 255])  # Upper bound in HSV

    # Mask for the brown color
    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    # Apply some morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("Mask", mask)
    if contours:
        # Find the largest contour (which is likely the box)
        largest_contour = max(contours, key=cv2.contourArea)

        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        # Check if the approximated contour has 4 points (likely a rectangle)
        if len(approx) == 4:
            # Draw the contour and bounding box on the frame
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Calculate the center of the box
            center_x = x + w // 2
            center_y = y + h // 2
            box_width = w

            # Determine the direction of the box relative to the robot
            frame_center_x = frame.shape[1] // 2
            if center_x < frame_center_x - 50:
                current_direction = DIRECTION.Left
            elif center_x > frame_center_x + 50:
                current_direction = DIRECTION.Right
            else:
                current_direction = DIRECTION.Ahead

            return (center_x, center_y, box_width), frame

    current_direction = DIRECTION.CannotFind
    return None, frame

def calculate_distance(box_width):
    """ Estimate distance to the box using the perceived width in pixels """
    return (KNOWN_WIDTH * FOCAL_LENGTH) / box_width

def gotoBoxSettings(box):
    '''
        Calls the vision system and determines the direction that the robot needs to move, 
        the distance to the box and if the robot will hit the box in the next move 

        INPUTS
            self: the box image captured from the camera

        OUTPUTS
            direction = direction that the robot is relative to the box
            speed = speed that the robot should travel at in the next time step
            pauseTime = how long the robot should travel at this speed for 
            noHit = whether the box will be hit by performing this movement 

        '''
    global robot_state, current_action

    if box is not None:
        center_x, center_y, box_width = box

        # Check if the box is centered
        if current_direction == DIRECTION.Left:
            print("Rotate left to center the box")
            current_action = ACTION.LEFT

        elif current_direction == DIRECTION.Right:
            print("Rotate right to center the box")
            current_action = ACTION.RIGHT

        elif current_direction == DIRECTION.Ahead:
            print("Box centered! Moving forward...")
            current_action = ACTION.FORWARD

            # Estimate the distance to the box
            distance = calculate_distance(box_width)
            print(f"Distance to box: {distance:.2f} cm")

            # When the robot reaches the box do a 180 degree rotation so it can deposit the balls.
            if distance < 5:  # Example threshold for stopping
                print("Box Reached: Performing rotation")
                robot_state = STATE.unloading
                current_action = ACTION.FORWARD
            return True

    else:
        print("Box not found. Rotating to find the box...")
        current_action = ACTION.RIGHT  # Rotate to search for the box
    return False

def find_and_goto_box():
    global robot_state

    robot_state = STATE.turn2Ball  # Start in this state

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # Find the box in the frame
        box, output_frame = find_box(frame)

        # Robot's decision logic based on current state
        if robot_logic(box):
            return

        # Display the result
        cv2.imshow("Box Detection", output_frame)
        
        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

