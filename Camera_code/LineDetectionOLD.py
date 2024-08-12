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
        


    def ImportImage(self): # import the image & crop it for more efficient processing 

        # read the image
        result, colour_image = self.cap.read()
        cv2.imwrite('OGimage.jpg', colour_image)

        # display the image on laptop FOR TESTING ONLY
        cv2.imshow("input", colour_image)

        # retrieve the height and width of the image 
        height, width = colour_image.shape[: 2]

        # crop image so that only bottom third is being analysed4
        crop_amount = 0.66 # percentage of image height to crop out
        height_min = int(height*crop_amount) # minimum pixel height 

        # crop the image
        cropped_image = colour_image[height_min:height, 0:width] 
        cv2.imwrite('cropped_image.jpg', cropped_image)

        return cropped_image # return the cropped image 


    def WhiteFilter(self, cropped_image:cv2.Mat)->cv2.Mat: # apply a mask that filters out non-white pixels 
        # define RGB upper and lower bounds for white 
        white_lower = np.array([150, 150, 150])
        white_upper = np.array([255, 255, 255])

        mask_white = cv2.inRange(cropped_image, white_lower, white_upper) # create a mask 
        white = cv2.bitwise_and(cropped_image,cropped_image,mask=mask_white) # filter out non white pixels
        cv2.imwrite('whiteimage.jpg', white)

        return white # return the balck & white image
    
    def EdgeDetection(self, white:cv2.Mat)->cv2.Mat: # apply edge detection and Guassian Blur
        # apply Guassian blur for noise reduction

        blurred = cv2.GaussianBlur(white, (15,15), 0)

        cv2.imwrite('blurredimage.jpg', blurred)

        # determine the threshold values for the Canny edge detection.
        # set the thresholds by computing the median of the pixel intensities 

        median_intensity = np.median(white)

        # set thresholds based on the median intensitiy 

        percentage = 0.33 # value to adjust the median intensity

        low_threshold = int(max(0, (1-percentage)*median_intensity)) # decreases median intensity by 33%
        high_threshold = int(min(255, (1+percentage)*median_intensity)) # increases median intensity by 33%

        # note that in greyscale, 0 is black and 255 is white

        # apply edge detection 
        edges = cv2.Canny(blurred, low_threshold, high_threshold)

        return edges # retrun the image with all the edges

    def HoughLineTransform(self, cropped_image:cv2.Mat, edges)->bool:
        # apply the Hough Line Transform 

        # set parameters 
        rho_set = 1
        theta_set = np.pi/180
        threshold = 200 # subject to adjustment

        lines = cv2.HoughLines(edges, rho_set, theta_set, threshold)

        if lines is None:
            flag = False # set flag to 0 -> no boundary line detected 
            
        else:
            flag = True # set flag to 1 to indicate that you are close to a boundary line 

            k = 1000; 

            for line in lines:

                # Find two points on the line so that a straight line can be drawn through it 
                rho, theta = line[0]
                
                a = np.cos(theta) 
                b = np.sin(theta)

                x0 = a*rho 
                y0 = b*rho

                x1 = int(x0 + k*(-b))
                y1 = int(y0 + k*(a))
                x2 = int(x0 - k*(-b))
                y2 = int(y0 - k*(a))

                # draw the boundary lines in red 
                cv2.line(cropped_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # save the image as a new file 
        cv2.imwrite('LineDetection.jpg', cropped_image)
        return flag


# Simple Script for Testing, do not use of Rpi

if __name__ == "__main__":
    from time import sleep
    Test = LineDetection(True)
    sleep(0.5) # wait for camera 

    while(True):
        # use esc to exit loop 
        key = cv2.waitKey(27)
        if key == 27: # ESC key pressed 
            break

        CroppedImage = Test.ImportImage()
        White = Test.WhiteFilter(CroppedImage)
        Edge = Test.EdgeDetection(White)
        flag = Test.HoughLineTransform(CroppedImage,Edge)

        if flag == True:
            print("At least one boundary line has been detected")
        else:
            print("No boundary line has been detected")
        sleep(0.1) #reduce load



