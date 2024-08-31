# Vision system code by Warren Rogan and Emma Vladicic

import cv2
from ECE4191enums import DIRECTION
import numpy as np


class Sys4_Vision:
    #Class variables
    greenLower = (29, 86, 30) # ball colour
    greenUpper = (50, 255, 255) # upper limit for the ball color first value used to be 64 
    known_radius = 0.03  # Tennis ball radius in m. Must be changed based on what sized tennis ball is being used. 
    focal_length = 1470  # Adjust based on camera's focal length (in pixels). Could not find on datasheet for the camera so might just need to tweak during testing to determine exact focal length

    #init
    def __init__(self, rpi: bool = True, tolerence: int =50 )-> None:
        self.tolerence = tolerence #tollerence of straight ahead in pixels
        self.rpi = rpi # define which OS is running
        if rpi:
            self.cap = cv2.VideoCapture(0) 
        else:
            self.cap =cv2.VideoCapture(1) #cv2.VideoCapture(0, cv2.CAP_DSHOW)
        result, image = self.cap.read() # get the first image 
        #cv2.imshow('test', image)    
        self.midpoint = image.shape[1]/2 # define where the middle of the image is 
        self.image = None
        self.aspcectRatio = Sys4_Vision.known_radius*Sys4_Vision.focal_length
        
    #detect
    def detect(self)->tuple[DIRECTION,bool,float]:
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

                if abs(center[0]-self.midpoint)<self.tolerence:
                    distance = self.aspcectRatio / radius #get the distance to the ball from the camera 
                    return (DIRECTION.Ahead,False,distance)
                elif center[0] > self.midpoint:
                    return (DIRECTION.Left, False, distance)
                else:
                    return (DIRECTION.Right, False, distance) 
            else:
                return (DIRECTION.CannotFind, False, distance) # allow for the case that there is no tennis ball in the frame 


        else:
            return (DIRECTION.CannotFind, False, distance) 


    def saveImage(self):
        cv2.imwrite('outputImage.jpg', self.image)
    #disconnect     
    def disconnect(self)->None:
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

