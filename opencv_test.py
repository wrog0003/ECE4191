'''
Based of of https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
'''
import cv2 as cv

print("hello world")

# define HSV values
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

cam = cv.VideoCapture(0)
result, image = cam.read()
if result:
    print("success")
    #cv.imshow("GeeksForGeeks", image)
    # resize image and apply mask
    [x,y]= image.shape[:2]
    print(x,y)
    frame = cv.resize(image,(round(x/2),round(y/2)))
    blurred = cv.GaussianBlur(frame, (11, 11), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, greenLower, greenUpper)
    # remove artefacts/ stray pixels
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
    cv.imshow(mask)

    cv.imwrite("GeeksForGeeks.png", image)
else :
    print("issues")