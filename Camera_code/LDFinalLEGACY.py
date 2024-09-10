import cv2
import numpy as py 

# TO DO; test on courts to find area of lines on averaage

class Vision_Lines:

    # class variables 

    def __init__(self, Rpi: bool = True) -> None: # initiate the constructor
        
        self.Rpi = Rpi

        # start video capture
        if Rpi: 
            self.cap = cv2.VideoCapture(1) # Use Rpi Camera
        else:
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # Use Laptop camera

        # capture first frame 
        result, colour_image = self.cap.read()

    def lineDetection(self)-> bool:

        # STEP 0: SETUP 

        Line = False # initially no line is detected
        _, original_image = self.cap.read()

        # STEP 1: CONVERT IMAGE TO GRAYSCALE AND KEEP WHITE OBJECTS

        # convert the image to greyscale 
        grey_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

        # define threshold values
        threshold = 170 
        max_value = 255 

        _, binary_image = cv2.threshold(grey_image, threshold, max_value, cv2.THRESH_BINARY) # now the image is purely black and white
        
        #cv2.imshow("input", binary_image)

        # STEP 2: PROCESS IMAGE 

        # retrieve the height and width of the image 
        #height, width = binary_image.shape[: 2]

        # crop image so that only bottom third is being analysed
        #crop_amount = 0.66 # percentage of image height to crop out THIS VALUE MIGHT NEED TO BE ADJUSTED WHEN CAMERA IS MOUNTED 
        #height_min = int(height*crop_amount) # minimum pixel height 

        # crop the image
        #cropped_image = binary_image[height_min:height, 0:width] 
        
        #cv2.imshow("input", cropped_image)

        # apply a Guassian Blur
        blurred_image = cv2.GaussianBlur(binary_image, (7,7), 0)
        
        #cv2.imshow("input", blurred_image)

        # use Canncy edge detecttion 
        edges = cv2.Canny(blurred_image, 50, 150) # last two values are the lower and upper thresholds of pixel intensity

        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # approximate the ontour to a polygon
        epsilon = 0.02*cv2.arcLength(largest_contour, True)

        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        # Draw the contours on the original frame
        cv2.drawContours(original_image, [approx], -1, (0, 255, 0), 3)
            
        cv2.imshow("input", original_image)
        Line = True
        # for contour in contours:
            
        #     # calcualte the area of the contour 
        #     area = cv2.contourArea(contour)

        #     print(area)

        #     #threshold = 0
            
        #     #if area > threshold:

        #         # line has been found 

        #     # approimate the contour to a polygon
        #     epsilon = 0.02*cv2.arcLength(contour, True)

        #     approx = cv2.approxPolyDP(contour, epsilon, True)

        #     # Draw the contours on the original frame
        #     cv2.drawContours(original_image, [approx], -1, (0, 255, 0), 3)
            
        #     cv2.imshow("input", original_image)

        #     # set a signal that says that line has been found 

        #     Line = True

        return Line 
        

# Simple Script for Testing, do not use of Rpi

if __name__ == "__main__":
    from time import sleep
    Test = Vision_Lines(False)
    sleep(0.5) # wait for camera 

    while(True):
        # use esc to exit loop 
        key = cv2.waitKey(27)
        if key == 27: # ESC key pressed 
            break

        Test.lineDetection()
                
        sleep(0.1) #reduce load



