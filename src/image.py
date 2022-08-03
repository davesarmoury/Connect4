import numpy as np
import cv2 as cv2
import math
from yaml import safe_load

def main():
    cam = cv2.VideoCapture(0, cv2.CAP_V4L2) # Need CAP_V4L2 for properties to work

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cam.set(cv2.CAP_PROP_AUTOFOCUS, False)
    cam.set(cv2.CAP_PROP_FOCUS, 0)
    cam.set(cv2.CAP_PROP_AUTO_WB, True)

    if not cam.isOpened():
        print("Cannot open camera")
        exit()

    while True:
        ret, frame = cam.read()
        cv2.imwrite("coins.jpg", frame)
        cv2.imshow("ass", frame)
        keu = cv2.waitKey(10)
        if keu & 0xFF == 27:
             break

main()
cv2.destroyAllWindows()
