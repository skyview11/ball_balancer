import cv2
import time
from cvDetection import *
import numpy as np

def draw_points(img, cnt, epsilon, color):
    cnt_length = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon * cnt_length, True)

    for point in approx:
        cv2.circle(img, (point[0][0], point[0][1]), 3, color, -1)


webcam = cv2.VideoCapture(cv2.CAP_DSHOW+1)
if not webcam.isOpened():
    print("Could not open webcam")
    exit()
t0 = time.time()
while webcam.isOpened():
    status, frame = webcam.read()
    
    if status:
        cv2.imshow("test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
webcam.release()
cv2.destroyAllWindows()