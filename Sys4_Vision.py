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
    lower_brown =(15, 40, 100)
    upper_brown = (50, 180, 255)
    #lowerWhite = (320,0,90) # lower limit for white line HSV colour scheme 
    #upperWhite = (360, 10, 100) # upper limit for white line HSV colour scheme 
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
            line_present = False#self.lineDetection()
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
        it follows the following method 
            1. converts the image to grey scale
            2. thresholds the image to check if the section is whiteish
            3. crops the image to reduce the amount of data
            4. gets the number of white pixels (blob stats)
            5. calculates the fraction of white pixels
            6. if more than 10% is white, then consider that is is close to a boundary '''

        # get  the image 
        original_image = self.image

        if not self.rpi:
            cv2.imshow("input", original_image) #display image ONLY for debugging

        # convert the image to greyscale 
        grey_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2HLS)

        white_lower = 180
        white_upper = 255

        _, binary_image = cv2.threshold(grey_image, white_lower, white_upper, cv2.THRESH_BINARY) # now the image is purely black and white
        #binary_image = cv2.inRange(original_image, white_lower, white_upper)
        # retrieve the height and width of the image 
        height, width = binary_image.shape[: 2]

        # crop image so that only bottom third is being analysed
        crop_amount = 0.66 # percentage of image height to crop out THIS VALUE MIGHT NEED TO BE ADJUSTED WHEN CAMERA IS MOUNTED 
        height_min = int(height*crop_amount) # minimum pixel height 

        # crop the image
        cropped_image = binary_image[height_min:height, 0:width] 

        # make the middle pixels black so that it does no accidently identify a tennis ball as a line 
        # want to crop out 1/8 of the width either side of the centre of the image 

        height_cropped, width_cropped = cropped_image.shape[: 2] # find the height and width of the cropped image
        centre_width = width_cropped/2 # determine the centre of image (width)

        # crop out 1/8 of the width either side of the centre line
        min_width = centre_width - 1/4*width_cropped
        max_width = centre_width + 1/4*width_cropped

        # round the width values 
        min_width = round(min_width)
        max_width = round(max_width)

        print(min_width)
        print(max_width)
        cropped_image [0:height_cropped, min_width:max_width] = 0 # set these values be black pixels
        
        if not self.rpi:
            cv2.imshow("cropped_image", cropped_image) # only white and black cropped image 

        # calculate the number of white pixels 
        white_pixels = np.sum(cropped_image == 255)

        # calculate total number of pixels 
        total_pixels = (height_cropped)*width_cropped

        # calculate average of white pixels 
        white_average = white_pixels/total_pixels

        # if the average of white pixels is inbetween 10% or less than 20% there is a boundary ahead 
        if 0.10 < white_average and white_average < 0.20 : 
            LineFound = True 
            print("Line detected")
        else:
            LineFound = False 
        if white_average >0.01:
            print(f'Line average ={white_average}\n')
        return LineFound

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
    looker = Sys4_Vision(False)
    sleep(0.5) # wait for camera
    
    while True:
        # use esc to exit loop 
        key = cv2.waitKey(1)
        if key == 27: #ESC Key to exit
            break
        
        result = looker.detect()
        result2 = looker.lineDetection()
        
        print(result)
        sleep(0.2)

    looker.disconnect() 

