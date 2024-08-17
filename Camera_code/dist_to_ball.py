import cv2
import numpy as np

def detect_tennis_ball(frame, min_radius=10):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the HSV color range for a tennis ball (adjust these values for better results)
    lower_yellow = np.array([25, 50, 50])
    upper_yellow = np.array([35, 255, 255])
    
    # Create a mask to filter out everything but the yellow color
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Perform some morphological operations to remove noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize variables
    center = None
    radius = 0
    
    # Only proceed if at least one contour was found
    if contours:
        # Find the largest contour in the mask, then use it to compute the minimum enclosing circle
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        
        # Only proceed if the radius meets a minimum size
        if radius > min_radius:
            # Draw the circle and centroid on the frame
            center = (int(x), int(y))
            cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
    
    return frame, center, radius

def calculate_distance_to_camera(known_radius, focal_length, perceived_radius):
    # Calculate the distance to the camera
    if perceived_radius > 0:
        distance = (known_radius * focal_length) / perceived_radius
        return distance
    return None

def main():
    # Constants
    known_radius = 3.4  # Tennis ball radius in cm
    focal_length = 700  # Adjust based on your camera's focal length (in pixels)
    
    # Start video capture
    cap = cv2.VideoCapture(0)
    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
        
            if not ret:
                break
        
            # Detect the tennis ball in the frame
            frame, center, radius = detect_tennis_ball(frame)
        
            # Calculate the distance from the ball to the camera
            distance = calculate_distance_to_camera(known_radius, focal_length, radius)
            print(f"Distance: {distance:.2f} cm")
            #if distance:
                # Display the distance on the frame
                #cv2.putText(frame, f"Distance: {distance:.2f} cm", (10, 30),
                            #cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
            # Display the resulting frame
            #cv2.imshow('Tennis Ball Detection', frame)
        
            # Break the loop on 'q' key press
            #if cv2.waitKey(1) & 0xFF == ord('q'):
                #break
    except KeyboardInterrupt:
        # Release the capture and close windows
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()