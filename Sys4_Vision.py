# Vision system code by Warren Rogan and Emma Vladicic

import cv2
from ECE4191enums import DIRECTION


class Sys4_Vision:
    #Class variables
    greenLower = (29, 86, 30) # ball colour
    greenUpper = (64, 255, 255)

    #init
    def __init__(self, rpi: bool = True, tolerence: int =30 )-> None:
        self.tolerence = tolerence
        self.rpi = rpi 
        if rpi:
            self.cap = cv2.VideoCapture(0) 
        else:
            self.cap =cv2.VideoCapture(0, cv2.CAP_DSHOW)

        result, image = self.cap.read()
        self.midpoint = image.shape[1]/2 
        
    #detect
    def detect(self)->tuple[DIRECTION,bool]:
        result, image = self.cap.read()
        if result:
            # blur to reduce artifacts
            blurred = cv2.GaussianBlur(image, (11, 11), 0)
            # filter image 
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, Vision.greenLower, Vision.greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            # get contors 
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0]

            # init center 
            center = None
            run = cnts != None and len(cnts)>0 
            if run:
                #get biggest shape
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c) # get the moments
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if not self.rpi: #is debugging on laptop
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2) # track perimeter of ball 
                    cv2.circle(image, center, 5, (0, 0, 255), -1) # marks centre 
                    cv2.imshow("frame",image)

                if abs(center[0]-self.midpoint)<self.tolerence:
                    return DIRECTION.Ahead
                elif center[0] < self.midpoint:
                    return DIRECTION.Left
                else:
                    return DIRECTION.Right
            else:
                return DIRECTION.CannotFind


        else:
            return DIRECTION.CannotFind

    #disconnect     
    def disconnect(self)->None:
        self.cap.release() 
        if not self.rpi:
            cv2.destroyWindow("Frame") 


# simple script for testing, do not use on rpi 
if __name__ == "__main__":
    from time import sleep
    looker = Vision(False)
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

