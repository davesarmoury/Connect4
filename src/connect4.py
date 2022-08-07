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
from threading import Thread, Lock

move_speed = 0.5
coin_radius = 20
color_tolerance = 30
minmax_depth = 6

home = [0,0,0,0,0,0]

def is_player(tk, color, color_tolerance):
    if np.all(tk <= color["max"]) and np.all(tk >= color["min"]):
        return True
    return False

def token_color(img, x, y):
    crop = img[ y - coin_radius : y + coin_radius, x - coin_radius : x + coin_radius ]
    return np.average(np.average(crop, axis=0), axis=0)

def get_calib_colors(calib_data):
    p_max = np.array([calib_data["player"]["max"]["b"], calib_data["player"]["max"]["g"], calib_data["player"]["max"]["r"]])
    p_min = np.array([calib_data["player"]["min"]["b"], calib_data["player"]["min"]["g"], calib_data["player"]["min"]["r"]])
    r_max = np.array([calib_data["robot"]["max"]["b"], calib_data["robot"]["max"]["g"], calib_data["robot"]["max"]["r"]])
    r_min = np.array([calib_data["robot"]["min"]["b"], calib_data["robot"]["min"]["g"], calib_data["robot"]["min"]["r"]])

    p_max = np.clip(p_max + color_tolerance, 0, 255)
    p_min = np.clip(p_min - color_tolerance, 0, 255)
    r_max = np.clip(r_max + color_tolerance, 0, 255)
    r_min = np.clip(r_min - color_tolerance, 0, 255)

    player_color = {}
    player_color["max"] = [p_max[0], p_max[1], p_max[2]]
    player_color["min"] = [p_min[0], p_min[1], p_min[2]]
    robot_color = {}
    robot_color["max"] = [r_max[0], r_max[1], r_max[2]]
    robot_color["min"] = [r_min[0], r_min[1], r_min[2]]

    return robot_color, player_color

def getBoardState(img, calib_data):
    state = np.zeros((6, 7))

    robot_color, player_color = get_calib_colors(calib_data)

    turn = 1

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

def cam_thread():
    global RUNNING, DO_MOVE, board_lock, board_state, turn_count, calib_data, cam

    r_color, p_color = get_calib_colors(calib_data)
    robot_color = [r_color["max"], r_color["min"]]
    robot_color = np.mean(robot_color, axis=0)
    player_color = [p_color["max"], p_color["min"]]
    player_color = np.mean(player_color, axis=0)

    while True:
        ret, frame = cam.read()

        temp_board_state, temp_turn_count = getBoardState(frame, calib_data)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

        for index, L in enumerate(calib_data["coins"]):
            x = L["x"]
            y = L["y"]

            row = int(index / 7)
            column = index % 7

            if temp_board_state[row][column] == 1: # Player
                cv2.circle(gray,(x,y),coin_radius*2,(player_color),-1)
            if temp_board_state[row][column] == 2: # Robot
                cv2.circle(gray,(x,y),coin_radius*2,(robot_color),-1)

        cv2.imshow("board", gray)
        key = cv2.waitKey(10)

        board_lock.acquire()
        board_state = temp_board_state
        turn_count = temp_turn_count
        board_lock.release()

        if key & 0xFF == 27:
            RUNNING = False
            break
        if key & 0xFF == 32:
            DO_MOVE = True

    cv2.destroyAllWindows()

def victory_dance():
    pass

def shame():
    pass

def main():
    global RUNNING, DO_MOVE, board_lock, board_state, turn_count, calib_data, cam
    calib_data = (safe_load(open("coins.yaml", "r")))

    #arm = MyCobot("/dev/ttyUSB0", 1000000)

    cam = cv2.VideoCapture(0, cv2.CAP_V4L2) # Need CAP_V4L2 for properties to work

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cam.set(cv2.CAP_PROP_AUTOFOCUS, False)
    cam.set(cv2.CAP_PROP_FOCUS, 0)
    cam.set(cv2.CAP_PROP_AUTO_WB, True)

    if not cam.isOpened():
        print("Cannot open camera")
        exit()

    board_lock = Lock()
    thread = Thread(target=cam_thread, args=())
    thread.daemon = True
    RUNNING = True
    DO_MOVE = False
    thread.start()

    #arm.sync_send_angles(home, move_speed)
    while RUNNING:
        if DO_MOVE:
            print("Processing move")

            board_lock.acquire()
            state = board_state
            turn = turn_count
            board_lock.release()

            board = Board(state, True)
            board.pretty_print()
            game = Game()
            game.turn = turn

            start_time = time.time()
            (column, confidence) = ai.minimax(game,board,minmax_depth,True)
            print(str(time.time() - start_time) + " seconds")
            print(column)
            print(confidence)

            if column == None: # Game Over
                if confidence > 0: # Win
                    victory_dance()
                else: # Lose
                    shame()

            DO_MOVE = False

        time.sleep(0.1)

main()
