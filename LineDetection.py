# Line Detection Script, 2/8/24 
# Script to find a line on a tennis ball court, using a test image from the Internet 
# Still need to work out how to interface this with the Pi Camera 

# source: https://www.geeksforgeeks.org/line-detection-python-opencv-houghline-method/

import cv2 
import numpy as np

# Step 1: import the image, convert to grayscale 
colour_image = cv2.imread('test.jpg')
image = cv2.cvtColor(colour_image, cv2.COLOR_BGR2GRAY)

# Step 2: Apply Canny edge detection

# determine the threshold values for the Canny edge detection.
# set the thresholds by computing the median of the pixel intensities 

median_intensity = np.median(image)

# set thresholds based on the median intensitiy 

percentage = 0.33 # value to adjust the median intensity

low_threshold = int(max(0, (1-percentage)*median_intensity)) # decreases median intensity by 33%
high_threshold = int(min(255, (1+percentage)*median_intensity)) # increases median intensity by 33%

# note that in greyscale, 0 is black and 255 is white

# apply edge detection 
edges = cv2.Canny(image, low_threshold, high_threshold)

# apply the Hough Line Transform 

# set parameters 
rho_set = 1
theta_set = np.pi/180
threshold = 200 # subject to adjustment

lines = cv2.HoughLines(edges, rho_set, theta_set, threshold)

for line in lines:
    rho, theta = line[0]

    a = np.cos(theta) 
    b = np.sin(theta)

    x0 = a*rho
    y0 = b*rho

    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(colour_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

cv2.imwrite('LineDetection.jpg', colour_image)












