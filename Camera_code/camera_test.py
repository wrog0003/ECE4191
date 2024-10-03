import cv2

class CameraTest:
    def __init__(self, rpi: bool = True)-> None:
        self.rpi = rpi # define which OS is running
        if rpi:
            self.cap = cv2.VideoCapture(0) 
        else:
            self.cap =cv2.VideoCapture(0, cv2.CAP_DSHOW) # Logitech webcam for ball and box detection
            self.cap2 = cv2.VideoCapture (1, cv2.CAP_DSHOW) # Pi Cam for line detection
            print("accesed")
    
    
    def test(self) -> None:

        image1 = self.cap.read()
        image2 = self.cap2.read()

        cv2.imwrite("Logitech_Cam", image1) 
        cv2.imwrite("Rpi_Cam", image2) 

if __name__ == "__main__":
    from time import sleep
    camera = CameraTest() 
    sleep(0.5) # wait for camera
    
    while True:
        # use esc to exit loop 
        key = cv2.waitKey(1)
        if key == 27: #ESC Key to exit
            break
        
        camera.test()
        
        print('in while loop')
        sleep(0.2)

    camera.disconnect() 
