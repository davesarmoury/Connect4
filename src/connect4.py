import numpy as np
import cv2 as cv2
import math
from yaml import safe_load
import numpy as np
import sys
sys.path.insert(0, 'Connect-Four-AI/Connect-Four')
from connect_four import Board, Game

move_speed = 0.5
color_range = 10
coin_radius = 10
color_tolerance = 40

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
    state = np.zeros((7, 6))

    player = (calib_data["player"]["b"], calib_data["player"]["g"], calib_data["player"]["r"])
    robot = (calib_data["robot"]["b"], calib_data["robot"]["g"], calib_data["robot"]["r"])

    for index, L in enumerate(calib_data["coins"]):
        tk = token_color(img, L["x"], L["y"])

        row = int(index / 7)
        column = index % 7
        if dist(tk, player) < color_tolerance:
            state[column][row] = 1
        if dist(tk, robot) < color_tolerance:
            state[column][row] = 2

    return state

def main():
    calib_data = (safe_load(open("coins.yaml", "r")))

    #arm = MyCobot("/dev/ttyUSB0", 1000000)

    #cam = cv2.VideoCapture(0)
    #if not cam.isOpened():
    #    print("Cannot open camera")
    #    exit()

    #arm.sync_send_angles(home, move_speed)
    #ret, frame = cam.read()
    frame = cv2.imread("coins.jpg")
    state = getBoardState(frame, calib_data)
    board = Board(state, True)
    board.pretty_print()

main()
