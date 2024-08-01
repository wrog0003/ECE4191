'''
Based of of https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv2/
Depeding on what platform you are using check TODO comments as they define setup
'''

import cv2
from time import sleep as sl

print("hello world")

# define HSV values
greenLower = (29, 86, 30) # ball colour
greenUpper = (64, 255, 255)

#cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # use for webcam TODO 
cap = cv2.VideoCapture(0) # use for rpi  TODO 
sl(1)
result, image = cap.read()
midpoint = image.shape[1]/2  
tollerence = 30 


while True:
    result, image = cap.read()

    key = cv2.waitKey(1)
    if key == 27: #ESC Key to exit
        break




    if result:
        print("success")

        frame = image 
        blurred = cv2.GaussianBlur(image, (11, 11), 0) 
        
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        # remove artefacts/ stray pixels
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        #cv2.imshow("fd",mask)
        
        center = None
        # only proceed if at least one contour was found
        run = cnts != None and len(cnts)>0 
        
        if run:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea) # gets the largest ball which should be the closest ball
            #((x, y), radius) = cv2.minEnclosingCircle(c) # define circle around shape
            M = cv2.moments(c) 
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) #define centre based on mass centre 
            # only proceed if the radius meets a minimum size
            if True:#radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                '''
                Only for debugging, remove in deplpoyment
                '''
                #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2) # track perimeter of ball 
                cv2.circle(frame, center, 5, (0, 0, 255), -1) # track centre 
            # define direction 
            '''
            Left is 320 >
            Right is 320< 
            '''
            print(center[0])
            if abs(center[0]-midpoint) <tollerence:
                print ("Ball is ahead")
            elif center[0] < midpoint:
                print("left")
            else :
                print("right")
           
            
        #cv2.imshow("fred",frame) # comment out if using rpi TODO 
        sl(0.2)

        
    else :
        print("issues")