# Vision system code by Warren Rogan and Emma Vladicic

import cv2
from ECE4191enums import DIRECTION
import numpy as np
import adafruit_tcs34725
import board
import busio

class Sys4_Vision:
    '''
    This class encapsulates all the functionality related to use of the camera.

    It uses OpenCV to access the camera and perform image operations. 
    '''
    #Class variables
    greenLower = (29, 86, 30) # ball colour
    greenUpper = (50, 255, 255) # upper limit for the ball color first value used to be 64 
    lower_brown =(15, 40, 50) #V was 100
    upper_brown = (50, 180, 255)
    known_radius = 0.03  # Tennis ball radius in m. Must be changed based on what sized tennis ball is being used. 
    focal_length = 1470  # Adjust based on camera's focal length (in pixels). Could not find on datasheet for the camera so might just need to tweak during testing to determine exact focal length
    
    # Parameters for the box 
    # /TODO NEED TO ADJUST FOR ACTUAL BOX DIMENSIONS
    boxLength = 0.10
    boxWidth = 0.20

    

    #init
    def __init__(self, rpi: bool = True, tolerance: int =50 )-> None:
        '''
        This function creates an instance of the vision class
        
        Inputs:
            self: a class instance
            rpi: whether or not this is running on the raspberry pi, use false for running on laptop, defaults to True
            tolerance: the number of pixels to the left or right that should be considered ahead, defaults to 50
            '''
        self.tolerance = tolerance 
        '''Number of pixels to the left or right of center that is considered ahead'''
        self.rpi = rpi # define which OS is running
        if rpi:
            self.cap = cv2.VideoCapture(0) 
        else:
            self.cap =cv2.VideoCapture(0, cv2.CAP_DSHOW) 
            print("accesed")
        result, image = self.cap.read() # get the first image 
        self.midpoint = image.shape[1]/2 # define where the middle of the image is 
        '''the halfway point of the image'''
        self.image = None 
        '''Variable to store the image captured '''
        self.aspcectRatioBall = Sys4_Vision.known_radius*Sys4_Vision.focal_length
        self.aspcectRatioBox = Sys4_Vision.boxLength*Sys4_Vision.focal_length

        
        
    #detect
    def detect(self)->tuple[DIRECTION,bool,float]:
        ''' This function determines data about the location of a tennis ball and if a line is close
        
        Outputs:
            The direction that the ball is located relative to the robot
            If a line is detected close to the robot
            the distance to the ball if ahead (m)
            
        This gets an image from the camera then does the following:
            1. Creates a mask of the location of the tennis ball based on color filtering
            2. Find the shapes in the mask
            3. Checks that it detected some shapes
            4. if shapes detected finds the center of mass of the largest shape
            5. if running on a Laptop annotates the image and displays it
            6. Determines where the ball is relative to the robot
            7. If ahead also determines the distance to the ball'''
        result, self.image = self.cap.read() # get image 
        distance = -1 # define as a non possible value 
        if result:
            # blur to reduce artifacts
            blurred = cv2.GaussianBlur(self.image, (11, 11), 0)
            # filter image 
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, Sys4_Vision.greenLower, Sys4_Vision.greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # get contours 
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0]

            # init center 
            center = None
            run = contours != None and len(contours)>0 # check if contour exists and is not empty

            # run line detection check 
            line_present = False #self.lineDetection() #self.lineDetection 

            if run:
                #get biggest shape
                c = max(contours, key=cv2.contourArea) 
                M = cv2.moments(c) # get the moments
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) # save the moments in terms of center of area
                ((x, y), radius) = cv2.minEnclosingCircle(c) # get the radius of the contour 
                if not self.rpi: #is debugging on laptop
                    #((x, y), radius) = cv2.minEnclosingCircle(c) 
                    cv2.circle(self.image, (int(x), int(y)), int(radius),(0, 255, 255), 2) # track perimeter of ball 
                    cv2.circle(self.image, center, 5, (0, 0, 255), -1) # marks centre 
                    cv2.imshow("Frame", self.image) # show the resulting image 
                
                if abs(center[0]-self.midpoint)<self.tolerance:
                    distance = self.aspcectRatioBall / radius #get the distance to the ball from the camera 
                    return (DIRECTION.Ahead, line_present ,distance)
                elif center[0] < self.midpoint:
                    return (DIRECTION.Left, line_present , distance)
                else:
                    return (DIRECTION.Right, line_present, distance) 
            else:
                return (DIRECTION.CannotFind, line_present, distance) # allow for the case that there is no tennis ball in the frame 


        else:
            return (DIRECTION.CannotFind, False, distance) 
        
    def detectBox(self)->tuple[DIRECTION,bool,float]:
        ''' This function determines data about the location of the box and if a line is close
        
        Outputs:
            The direction that the box is located relative to the robot
            If a line is detected close to the robot
            the distance to the box if ahead (m)
            
        This gets an image from the camera then does the following:
            1. Creates a mask of the location of the box based on color filtering
            2. Find the shapes in the mask
            3. Checks that it detected some shapes
            4. if shapes detected finds the center of mass of the largest shape
            5. if running on a Laptop annotates the image and displays it
            6. Determines where the box is relative to the robot
            7. If ahead also determines the distance to the box'''
        
        result, frame = self.cap.read() # Captures the image
        distance = -1 # define as a non possible value 
        if result:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Mask for the brown color
            mask = cv2.inRange(hsv, Sys4_Vision.lower_brown, Sys4_Vision.upper_brown)

            # Apply some morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not self.rpi: # show image if running on laptop 
                cv2.imshow("Mask", mask)

            # run line detection check 
            line_present = self.lineDetection()
            #line_present = 0
            if contours:
                # Find the largest contour (which is likely the box)
                largest_contour = max(contours, key=cv2.contourArea)

                # Approximate the contour to a polygon
                epsilon = 0.02 * cv2.arcLength(largest_contour, True)
                approx = cv2.approxPolyDP(largest_contour, epsilon, True)

                # Check if the approximated contour has 4 points (likely a rectangle) 
                #TODO consider that a box may not be 4 if vied from any other angle that head on 
                if len(approx) >=4 or len(approx) <=8:
                    # Draw the contour and bounding box on the frame
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Calculate the center of the box
                    center_x = x + w // 2
                    center_y = y + h // 2
                    box_width = w

                    # Determine the direction of the box relative to the robot
                    frame_center_x = frame.shape[1] // 2

                    # Determine the distance to the Box 
                    distance = self.aspcectRatioBox / w #get the distance to the box from the camera
                    if not self.rpi: # show image if running on laptop 
                        cv2.imshow("Image", frame)
                    # Determine the direction to the box and return the direction, if there is a line present and the distance to the box 
                    if center_x < frame_center_x - 50:
                        return (DIRECTION.Left, line_present, distance)
                    elif center_x > frame_center_x + 50:
                        return (DIRECTION.Right, line_present, distance)
                    else:
                        return (DIRECTION.Ahead, line_present, distance)

            else: 
                distance = -1 # Box could not be found 
                return (DIRECTION.CannotFind, line_present, distance)
            


    def lineDetection(self)-> bool:
        '''Returns if a line was detected
        '''

        # Parameters/setup for colour sensor
        # set up I2C communication with the sensor
        i2c = busio.I2C(board.SCL, board.SDA)

        # create a colour sensor oject
        sensor = adafruit_tcs34725.TCS34725(i2c)

        # enable the sensor's internal LED for better colour reading 
        sensor.enable_led = True
        # get the RGB values that the sensor is reading 

        r, g, b, c = sensor.color_raw 

        # clear refers to the measurement of total light intensity falling on the sensor

        print(f"Raw Colour Data - Red:{r}, Green:{g}, Blue: {b}, Clear:{c}")

        if c > 130: # This is a Line
            return True
        else: 
            return False
        

    def saveImage(self):
        '''Saves the image as outputImage.jpg so that you can look at the camera feed'''
        cv2.imwrite('outputImage.jpg', self.image)
    #disconnect     
    def __del__(self)->None:
        '''Disconnects the camera and removes the image window if running on laptop'''
        self.cap.release() 
        if not self.rpi:
            cv2.destroyWindow("Frame") 


# simple script for testing, do not use on rpi 


if __name__ == "__main__":
    from time import sleep
    looker = Sys4_Vision(True)
    sleep(0.5) # wait for camera
    
    while True:
        # use esc to exit loop 
        key = cv2.waitKey(1)
        if key == 27: #ESC Key to exit
            break
        
        #result = looker.detect()
        result2 = looker.lineDetection()
        #result = looker.detectBox()
        print(result2)
        sleep(0.2)

    looker.disconnect() 

