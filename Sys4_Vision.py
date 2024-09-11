# Vision system code by Warren Rogan and Emma Vladicic

import cv2
from ECE4191enums import DIRECTION
import numpy as np


class Sys4_Vision:
    '''
    This class encapsulates all the functionality related to use of the camera.

    It uses OpenCV to access the camera and perform image operations. 
    '''
    #Class variables
    greenLower = (29, 86, 30) # ball colour
    greenUpper = (50, 255, 255) # upper limit for the ball color first value used to be 64 
    known_radius = 0.03  # Tennis ball radius in m. Must be changed based on what sized tennis ball is being used. 
    focal_length = 1470  # Adjust based on camera's focal length (in pixels). Could not find on datasheet for the camera so might just need to tweak during testing to determine exact focal length

    #init
    def __init__(self, rpi: bool = True, tolerance: int =50 )-> None:
        '''
        This function creates an instance of the vision class
        
        Inputs:
            self: a class instance
            rpi: whether or not this is running on the raspberry pi, use false for running on laptop, defaults to True
            tolerance: the number of pixels to the left or right that should be considered ahead, defaults to 50
            '''
        self.tolerance = tolerance #tolerance of straight ahead in pixels
        '''Number of pixels to the left or right of center that is considered ahead'''
        self.rpi = rpi # define which OS is running
        if rpi:
            self.cap = cv2.VideoCapture(0) 
        else:
            self.cap =cv2.VideoCapture(1) #cv2.VideoCapture(0, cv2.CAP_DSHOW)
        result, image = self.cap.read() # get the first image 
        
        self.midpoint = image.shape[1]/2 # define where the middle of the image is 
        self.image = None
        self.aspcectRatio = Sys4_Vision.known_radius*Sys4_Vision.focal_length

        # Parameters for the box 
        self.boxLength = 10
        self.boxWidth = 20
        
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

            # get contors 
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0]

            # init center 
            center = None
            run = cnts != None and len(cnts)>0 # check if contour exists and is not empty

            # run line detection check 
            line_present = self.lineDetection 

            if run:
                #get biggest shape
                c = max(cnts, key=cv2.contourArea) 
                M = cv2.moments(c) # get the moments
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) # save the moments in terms of center of area
                ((x, y), radius) = cv2.minEnclosingCircle(c) # get the radius of the contour 
                if not self.rpi: #is debugging on laptop
                    #((x, y), radius) = cv2.minEnclosingCircle(c) 
                    cv2.circle(self.image, (int(x), int(y)), int(radius),(0, 255, 255), 2) # track perimeter of ball 
                    cv2.circle(self.image, center, 5, (0, 0, 255), -1) # marks centre 
                    cv2.imshow("Frame", self.image) # show the resulting image 
                
                if abs(center[0]-self.midpoint)<self.tolerance:
                    distance = self.aspcectRatio / radius #get the distance to the ball from the camera 
                    return (DIRECTION.Ahead, line_present ,distance)
                elif center[0] > self.midpoint:
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

            # run line detection check 
            line_present = self.lineDetection

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

                    # Determine the distance to the Box 
                    distance = self.aspcectRatio / self.boxLength #get the distance to the box from the camera

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

        # get  the image 
        original_image = self.image

        # cv2.imshow("input", original_image) display image ONLY for debugging

        # convert the image to greyscale 
        grey_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

        # define threshold values for white 
        threshold = 180 
        max_value = 255 

        _, binary_image = cv2.threshold(grey_image, threshold, max_value, cv2.THRESH_BINARY) # now the image is purely black and white

        # retrieve the height and width of the image 
        height, width = binary_image.shape[: 2]

        # crop image so that only bottom third is being analysed
        crop_amount = 0.33 # percentage of image height to crop out THIS VALUE MIGHT NEED TO BE ADJUSTED WHEN CAMERA IS MOUNTED 
        height_max = int(height*crop_amount) # minimum pixel height 

        # crop the image
        cropped_image = binary_image[0:height_max, 0:width] 

        # cv2.imshow("cropped_image", cropped_image) onlly 

        # calculate the number of white pixels 
        white_pixels = np.sum(cropped_image == 255)

        # calculate total number of pixels 
        total_pixels = (height_max)*width

        # calculate average of white pixels 
        white_average = white_pixels/total_pixels

        # if the average of white pixels is inbetween 10-35% then there is a boundary ahead 
        if 0.10 < white_average < 0.35:
            LineFound = True 
        else:
            LineFound = False 
        
        return LineFound


    def saveImage(self):
        cv2.imwrite('outputImage.jpg', self.image)
    #disconnect     
    def disconnect(self)->None:
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
        result = looker.detect()
        
        print(result)
        sleep(0.2)

    looker.disconnect() 

