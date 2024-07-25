print("hello world")
import cv2 as cv

cam = cv.VideoCapture(0)
result, image = cam.read()
if result:
    print("success")
    #cv.imshow("GeeksForGeeks", image)
    cv.imwrite("GeeksForGeeks.png", image)
else :
    print("issues")