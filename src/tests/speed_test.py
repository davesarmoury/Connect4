import sys
sys.path.insert(0, '../Connect-Four-AI/Connect-Four')
from connect_four import Board, Game
import connect_four_ai as ai
import json
import time
from tqdm import tqdm

board_file = open("boards.json", "r")
test_boards = (json.load(board_file))
board_file.close()

print("Found " + str(len(test_boards)) + " boards")

for depth in range(1, 7):
    avv = 0
    for board_moves in tqdm(test_boards):
        board = Board()
        game = Game()
        for move in board_moves:
            chip = game.create_chip()
            chip.drop_chip(board, int(move))

        start_time = time.time()
        col = ai.minimax(game, board, depth, True)
        end_time = time.time()
        time_diff = end_time - start_time
        avv = avv + time_diff

    avv = avv / float(len(test_boards))

    print(str(depth) + " - " + str(avv))
