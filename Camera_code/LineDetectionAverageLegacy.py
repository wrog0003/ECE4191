# Line Detection Script, Emma Vladicic
# Last edit: 5/8/24 : Made this into a class for better readability and functionality 
# Script to find a boundary line on a tennis ball court, using a test images from the Internet 

# Uses live feed from desktop camera and tries to find lines
# TO DO 
# connect the Rpi camera, take it to the tennis courts and actually test it   

# Testing Notes
# Parameters that might need to be changed depending on how testing on the tennis court goes 
# crop amount, 
# Source: https://www.geeksforgeeks.org/line-detection-python-opencv-houghline-method/
# with help from https://chatgpt.com/c/3e4f3e64-92c7-4627-8ba4-9b14a0de68a8

import cv2 
import numpy as np

# define a class called LineDetection
class LineDetection:

    def __init__(self, Rpi: bool = True) -> None: # initiate the constructor
        self.Rpi = Rpi

        # start video capture

        if Rpi: 
            self.cap = cv2.VideoCapture(1) # to use rpi camera not rpi computer 
        else:
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

        # capture first frame 
        result, colour_image = self.cap.read()

    def WhiteAverager(self):

        # read the image 
        _, original_image = self.cap.read() # the _ at the start means the variable is not used 
        cv2.imshow("input", original_image)


        # convert the image to greyscale 
        grey_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

        # define threshold values
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
        cv2.imwrite('cropped_image.jpg', cropped_image)

        # calculate the number of white pixels 
        white_pixels = np.sum(cropped_image == 255)

        # calculate total number of pixels 
        total_pixels = (height-height_min)*width

        # calculate average of white pixels 
        white_average = white_pixels/total_pixels

        print("The average n.o white pixels is %.2f" % white_average)


        # if the average of white pixels is inbetween 8-12% then there is a boundary ahead 
        if 0.08 < white_average < 0.15:
            print("BOUNDAY LINE FOUND")
        

        

# Simple Script for Testing, do not use of Rpi

if __name__ == "__main__":
    from time import sleep
    Test = LineDetection(False)
    sleep(0.5) # wait for camera 

    while(True):
        # use esc to exit loop 
        key = cv2.waitKey(27)
        if key == 27: # ESC key pressed 
            break

        Test.WhiteAverager()
                
        sleep(0.1) #reduce load



