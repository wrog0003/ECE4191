# Vision system code by Warren Rogan and Emma Vladicic

import cv2
from ECE4191enums import DIRECTION
import numpy as np


class SysD_Camera:
    '''
    This class encapsulates all the functionality related to use of the camera.

    It uses OpenCV to access the camera and perform image operations. 
    '''
    #Class variables
    greenLower = (29, 86, 30) # ball colour
    greenUpper = (50, 255, 255) # upper limit for the ball color first value used to be 64 
    lower_brown =(15, 40, 100) # box colour 
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
        self.rpi = rpi 
        '''Define if running on the pi or on a laptop'''
        if rpi:
            self.cap = cv2.VideoCapture(0) 
        else:
            self.cap =cv2.VideoCapture(0, cv2.CAP_DSHOW) 
            print("accessed")
        result, image = self.cap.read() # get the first image 
        self.midpoint = image.shape[1]/2 # define where the middle of the image is 
        '''the halfway point of the image'''
        self.image = None 
        '''Variable to store the image captured '''
        self.aspcectRatioBall = SysD_Camera.known_radius*SysD_Camera.focal_length
        '''Aspect ratio for pixel size for the tennis ball'''
        self.aspcectRatioBox = SysD_Camera.boxLength*SysD_Camera.focal_length
        '''Aspect ratio for pixel size of the cardboard box'''
        
    #detect
    def detect(self,Ball:bool = True)->tuple[DIRECTION,bool,float]:
        ''' This function determines data about the location of a tennis ball and if a line is close
        Inputs:
            Ball: if looking for a ball or the box

        Outputs:
            The direction that the target is located relative to the robot
            If a line is detected close to the robot
            the distance to the target if ahead (m)
            
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
            if Ball:
                mask = cv2.inRange(hsv, SysD_Camera.greenLower, SysD_Camera.greenUpper) # filter for green
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)
            else:
                mask = cv2.inRange(hsv, SysD_Camera.lower_brown, SysD_Camera.upper_brown) # filter for brown
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # get contours 
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0]

            # init center 
            center = None
            run = contours != None and len(contours)>0 # check if contour exists and is not empty
            if not self.rpi:
                cv2.imshow("Mask",mask)

            # run line detection check 
            line_present = self.lineDetection() #self.lineDetection 

            if run:
                #get biggest shape
                largestContour = max(contours, key=cv2.contourArea) 
                if Ball:
                    M = cv2.moments(largestContour) # get the moments
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) # save the moments in terms of center of area
                    center_x = center[0]
                    ((x, y), radius) = cv2.minEnclosingCircle(largestContour) # get the radius of the contour 
                    if not self.rpi: #is debugging on laptop
                        #((x, y), radius) = cv2.minEnclosingCircle(largestContour) 
                        cv2.circle(self.image, (int(x), int(y)), int(radius),(0, 255, 255), 2) # track perimeter of ball 
                        cv2.circle(self.image, center, 5, (0, 0, 255), -1) # marks centre 
                        cv2.imshow("Frame", self.image) # show the resulting image 
                else: #box
                    # Approximate the contour to a polygon
                    epsilon = 0.02 * cv2.arcLength(largestContour, True)
                    approx = cv2.approxPolyDP(largestContour, epsilon, True)
                    if len(approx) >=4 or len(approx) <=8: # if number matches an approximation of a box 
                        x, y, w, h = cv2.boundingRect(approx)
                        # Calculate the center of the box
                        center_x = x + w // 2
                        center_y = y + h // 2
                        box_width = w
                    else: return (DIRECTION.CannotFind, line_present, distance) # if the shape does not match the box 

            
                if abs(center_x-self.midpoint)<self.tolerance: # ahead
                    if Ball:
                        distance = self.aspcectRatioBall / radius #get the distance to the ball from the camera 
                    else:
                        distance = self.aspcectRatioBox/w; 
                    return (DIRECTION.Ahead, line_present ,distance)
                
                elif center_x < self.midpoint:
                    return (DIRECTION.Left, line_present , distance)
                
                else:
                    return (DIRECTION.Right, line_present, distance) 
                
            else:
                return (DIRECTION.CannotFind, line_present, distance) # allow for the case that there is no target in the frame


        else:
            return (DIRECTION.CannotFind, False, distance)  # if couldn't get image feed
        
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
        grey_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

        # define threshold values for white 
        threshold = 180 
        max_value = 255 

        _, binary_image = cv2.threshold(grey_image, threshold, max_value, cv2.THRESH_BINARY) # now the image is purely black and white

        # retrieve the height and width of the image 
        height, width = binary_image.shape[: 2]

        # crop image so that only bottom third is being analysed
        crop_amount = 0.66 # percentage of image height to crop out THIS VALUE MIGHT NEED TO BE ADJUSTED WHEN CAMERA IS MOUNTED 
        height_min = int(height*crop_amount) # minimum pixel height 

        # crop the image
        cropped_image = binary_image[height_min:height, 0:width] 

        # make the middle pixels black so that it does no accidently identify a tennis ball as a line 
        # want to crop out 1/8 of the width either side of the centre of the image 

        height_cropped, width_cropped = cropped_image.shape[:,2] # find the height and width of the cropped image
        centre_width = width_cropped/2 # determine the centre of image (width)

        # crop out 1/8 of the width either side of the centre line
        min_width = centre_width - 1/8*width_cropped 
        max_width = centre_width + 1/8*width_cropped 

        cropped_image [0: height_cropped, min_width:max_width] = (0,0,0) # set these values be black pixels
        
        if not self.rpi:
            cv2.imshow("cropped_image", cropped_image) # only white and black cropped image 

        # calculate the number of white pixels 
        white_pixels = np.sum(cropped_image == 255)

        height_new = height - height_min
        # calculate total number of pixels 
        total_pixels = (height_new)*width

        # calculate average of white pixels 
        white_average = white_pixels/total_pixels

        # if the average of white pixels is inbetween 10% or greater then there is a boundary ahead 
        if 0.10 < white_average : # <0.35
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

    def __str__(self) -> str:
        '''Returns the string representation of the system'''
        if self.rpi:
            return f'Camera running on raspberry pi'
        else:
            return f'Camera running on laptop'
    #disconnect     
    def __del__(self)->None:
        '''Disconnects the camera and removes the image window if running on laptop'''
        self.cap.release() 
        if not self.rpi:
            cv2.destroyWindow("Frame") 


# simple script for testing, do not use on rpi 


if __name__ == "__main__":
    from time import sleep
    looker = SysD_Camera(False)
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

