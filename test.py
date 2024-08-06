import cv2

#input your device number here.  For me it was 2.
cap = cv2.VideoCapture(1)

while True:
    ret, img = cap.read()
    cv2.imshow("input", img)

    key = cv2.waitKey(1)
    if key == 27: #ESC Key to exit
        break

cap.release()
cv2.destroyAllWindows()