import cv2
from imutils.video import VideoStream
import time

# Initialize the USB webcam
webcam = VideoStream(src=0).start()  # Usually, 'src=0' is the webcam

# Initialize the Raspberry Pi camera
picam = VideoStream(usePiCamera=True).start()

# Allow both cameras to warm up
time.sleep(2.0)

try:
    while True:
        # Grab the frames from both cameras
        frame_webcam = webcam.read()
        frame_picam = picam.read()

        # Display the frames in separate windows
        cv2.imshow("Webcam", frame_webcam)
        cv2.imshow("Pi Camera", frame_picam)

        # Break out of the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop both video streams
    webcam.stop()
    picam.stop()

    # Close all OpenCV windows
    cv2.destroyAllWindows()