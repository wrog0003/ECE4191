import cv2
import numpy as np
from ECE4191enums import STATE, DIRECTION, ACTION  # Importing Enums
import time

global robot_state

# GPIO Setup for limit switch
LIMIT_SWITCH_PIN = 17  # Adjust according to your setup

# Setup camera
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Assuming you're using the default camera

# Known width of the line in the real world (in cm)
KNOWN_WIDTH = 2.0  # Adjust to the actual width of the line

# Focal length (can be calculated based on some sample images)
FOCAL_LENGTH = 615  # Adjust based on your camera specs

# Current state of the robot
robot_state = STATE.null
current_direction = DIRECTION.CannotFind
current_action = ACTION.FORWARD

def find_line(frame):
    """ Detect the white line in the frame using grayscale and binary thresholding """
    global current_direction

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Define the threshold range for detecting white
    white_lower = 235  # Adjust based on lighting conditions
    white_upper = 255

    # Apply binary thresholding to detect white regions
    _, binary_image = cv2.threshold(gray, white_lower, white_upper, cv2.THRESH_BINARY)

    # Apply some morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
    binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

    # Find contours in the binary image
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("Binary Image", binary_image)

    if contours:
        # Find the largest contour (which is likely the line)
        largest_contour = max(contours, key=cv2.contourArea)

        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        # Draw the contour and bounding box on the frame
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Calculate the center of the line
        center_x = x + w // 2
        line_width = w

        # Determine the direction of the line relative to the robot
        frame_center_x = frame.shape[1] // 2
        if center_x < frame_center_x - 50:
            current_direction = DIRECTION.Left
        elif center_x > frame_center_x + 50:
            current_direction = DIRECTION.Right
        else:
            current_direction = DIRECTION.Ahead

        return (center_x, line_width), frame

    current_direction = DIRECTION.CannotFind
    return None, frame

def calculate_distance(line_width):
    """ Estimate distance to the line using the perceived width in pixels """
    return (KNOWN_WIDTH * FOCAL_LENGTH) / line_width

def robot_logic(line):
    '''
        Calls the vision system and determines the direction that the robot needs to move, 
        the distance to the line and if the robot will reach the line in the next move.
    '''
    global robot_state, current_action

    if line is not None:
        center_x, line_width = line

        # Check if the line is centered
        if current_direction == DIRECTION.Left:
            print("Rotate left to center the line")
            current_action = ACTION.LEFT

        elif current_direction == DIRECTION.Right:
            print("Rotate right to center the line")
            current_action = ACTION.RIGHT

        elif current_direction == DIRECTION.Ahead:
            print("Line centered! Moving forward...")
            current_action = ACTION.FORWARD

            # Estimate the distance to the line
            distance = calculate_distance(line_width)
            print(f"Distance to line: {distance:.2f} cm")

            # When the robot reaches the line, it could stop or perform an action
            if distance < 5:  # Example threshold for stopping
                print("Line Reached: Performing stop or action")
                robot_state = STATE.unloading
                current_action = ACTION.FORWARD
            return True

    else:
        print("Line not found. Rotating to find the line...")
        current_action = ACTION.RIGHT  # Rotate to search for the line
    return False

if __name__ == "__main__":
    robot_state = STATE.turn2Ball  # Start in this state

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # Find the line in the frame
        line, output_frame = find_line(frame)

        # Robot's decision logic based on current state
        if robot_logic(line):
            pass

        # Display the result
        cv2.imshow("Line Detection", output_frame)
        
        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
