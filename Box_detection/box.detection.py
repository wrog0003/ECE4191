import cv2
import numpy as np
#import RPi.GPIO as GPIO
#from gpiozero import Button
import time

# GPIO Setup for limit switch
LIMIT_SWITCH_PIN = 17  # Adjust according to your setup
#limit_switch = Button(LIMIT_SWITCH_PIN)

# Setup camera
cap = cv2.VideoCapture(0)  # Assuming you're using the default camera

# Known width of the box in real world (in cm)
KNOWN_WIDTH = 30.0  # Adjust to the actual width of the box

# Focal length (can be calculated based on some sample images)
# Focal length in pixels, based on a 720p resolution and 4mm focal length
# You might want to fine-tune this based on real-world testing
FOCAL_LENGTH = 615  # Adjust based on your camera specs

def find_box(frame):
    """ Detect the cardboard box in the frame """
    # Convert to HSV color space to detect the brown color of the box
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the brown color range
    lower_brown = np.array([10, 100, 20])
    upper_brown = np.array([20, 255, 200])

    # Mask for the brown color
    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    # Apply some morphological operations to reduce noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

            return (center_x, center_y, box_width), frame

    return None, frame

def calculate_distance(box_width):
    """ Estimate distance to the box using the perceived width """
    # Distance calculation based on known width and focal length
    return (KNOWN_WIDTH * FOCAL_LENGTH) / box_width

def main():
    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        # Find the box in the frame
        box, output_frame = find_box(frame)

        if box:
            center_x, center_y, box_width = box

            # Get the width of the frame
            frame_center_x = frame.shape[1] // 2

            # Check if the box is centered
            if abs(center_x - frame_center_x) > 30:  # Adjust the threshold
                if center_x < frame_center_x:
                    print("Rotate left to center the box")
                else:
                    print("Rotate right to center the box")
            else:
                print("Box centered!")

            # Estimate distance
            distance = calculate_distance(box_width)
            print(f"Distance to box: {distance:.2f} cm")

            # If distance is small enough, check the limit switch
            if distance < 20:  # Example threshold for stopping
                print("Moving forward towards the box...")
                #if limit_switch.is_pressed:
                    #print("Limit switch pressed! Stopping the robot.")
                    #break

        # Display the result
        cv2.imshow("Box Detection", output_frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
