import json
import random

board_strings = []

for n in range(5):
  for i in range(1, 25):
    strrr = ""
    for x in range(i):
      strrr = strrr + str(random.randint(0, 6))
    if strrr not in board_strings:
        board_strings.append(strrr)

outFile = open("boards.json", 'w')
outFile.write(json.dumps(board_strings))
outFile.close()
