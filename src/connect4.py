import numpy as np
import cv2 as cv2
import math
from yaml import safe_load
import numpy as np
import sys
sys.path.insert(0, 'Connect-Four-AI/Connect-Four')
from connect_four import Board, Game
import connect_four_ai as ai
import time

move_speed = 0.5
coin_radius = 20
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

def is_player(tk, color, color_tolerance):
    if np.all(tk <= color["max"]) and np.all(tk >= color["min"]):
        return True
    return False

def token_color(img, x, y):
    crop = img[ y - coin_radius : y + coin_radius, x - coin_radius : x + coin_radius ]
    return np.average(np.average(crop, axis=0), axis=0)

def getBoardState(img, calib_data):
    state = np.zeros((6, 7))
    turn = 1

    player_color = {}
    player_color["max"] = (calib_data["player"]["max"]["b"], calib_data["player"]["max"]["g"], calib_data["player"]["max"]["r"])
    player_color["min"] = (calib_data["player"]["min"]["b"], calib_data["player"]["min"]["g"], calib_data["player"]["min"]["r"])
    robot_color = {}
    robot_color["max"] = (calib_data["robot"]["max"]["b"], calib_data["robot"]["max"]["g"], calib_data["robot"]["max"]["r"])
    robot_color["min"] = (calib_data["robot"]["min"]["b"], calib_data["robot"]["min"]["g"], calib_data["robot"]["min"]["r"])

    for index, L in enumerate(calib_data["coins"]):
        tk = token_color(img, L["x"], L["y"])

        row = int(index / 7)
        column = index % 7

        if is_player(tk, player_color, color_tolerance):
            state[row][column] = 1
            turn = turn + 1
        if is_player(tk, robot_color, color_tolerance):
            state[row][column] = 2
            turn = turn + 1

    return state, turn

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
    state, turn = getBoardState(frame, calib_data)
    board = Board(state, True)
    game = Game()
    game.turn = turn
    board.pretty_print()
    start_time = time.time()
    col = ai.minimax(game,board,5,True)
    print(str(time.time() - start_time) + " seconds")
    print(col)

main()
