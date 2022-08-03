import numpy as np
import cv2 as cv2
import math
from yaml import safe_load

def main():
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        print("Cannot open camera")
        exit()

    while True:
        ret, frame = cam.read()
        cv2.imwrite("/home/davesarmoury/board.jpg", frame)
        cv2.imshow("ass", frame)
        keu = cv2.waitKey(1)
        if keu & 0xFF == 27:
             break

main()
cv2.destroyAllWindows()
