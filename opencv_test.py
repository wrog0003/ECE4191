'''
Based of of https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv2/
'''
import cv2
from time import sleep as sl

print("hello world")

# define HSV values
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
fed = 1
while fed ==1:
    result, image = cap.read()

    key = cv2.waitKey(1)
    if key == 27: #ESC Key to exit
        break




    if result:
        print("success")
        #cv2.imshow("GeeksForGeeks", image)
        # apply mask
        frame = image 
        blurred = cv2.GaussianBlur(image, (11, 11), 0)
        
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        # remove artefacts/ stray pixels
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]

        center = None
        # only proceed if at least one contour was found
        run = cnts != None and len(cnts)>0 

        if run:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # update the points queue
            
            cv2.imwrite("fred",frame)
        fed = fed +1 
        sl(0.2)

        
    else :
        print("issues")