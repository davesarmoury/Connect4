import numpy as np
import cv2 as cv2
import math
from yaml import safe_load

move_speed = 0.5
color_range = 10
coin_radius = 10
color_tolerance = 20

home = [0,0,0,0,0,0]
slots = []
slots.append([0,0,0,0,0,0])
slots.append([0,0,0,0,0,0])
slots.append([0,0,0,0,0,0])
slots.append([0,0,0,0,0,0])
slots.append([0,0,0,0,0,0])
slots.append([0,0,0,0,0,0])
slots.append([0,0,0,0,0,0])

def dist(c1, c2):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2 + (c1[2] - c2[2]) ** 2)

def token_color(img, x, y):
    crop = img[ y - coin_radius : y + coin_radius, x - coin_radius : x + coin_radius ]
    return np.average(np.average(crop, axis=0), axis=0)

def getBoardState(img, calib_data):
    state = []

    player = (calib_data["player"]["b"], calib_data["player"]["g"], calib_data["player"]["r"])
    robot = (calib_data["robot"]["b"], calib_data["robot"]["g"], calib_data["robot"]["r"])

    for L in calib_data["coins"]:
        tk = token_color(img, L["x"]), L["y"])

        if dist(tk, player) < color_tolerance:
            state.append(2)
        elif dist(tk, robot) < color_tolerance:
            state.append(1)
        else:
            state.append(0)

    return getBoardState

def main():
    calib_data = (safe_load(open("coins.yaml", "r")))

    arm = MyCobot("/dev/ttyUSB0", 1000000)

    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        print("Cannot open camera")
        exit()

    arm.sync_send_angles(home, move_speed)
    ret, frame = cam.read()

    state = getBoardState(frame, calib_data)
