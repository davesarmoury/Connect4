import numpy as np
import cv2 as cv2

from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

calib_data = {"coins":[]}

img = cv2.imread("coins.jpg")
img_show = np.copy(img)

def draw_circle(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img_show,(x,y),40,(0,255,0),-1)
        calib_data["coins"].append({"x" : x,"y" : y})

cv2.imshow("image", img_show)
cv2.setMouseCallback('image',draw_circle)

while(1):
    cv2.imshow('image',img_show)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break

robot_color_averages = []
player_color_averages = []

c_radius = 10
for i, c in enumerate(calib_data["coins"]):
    crop = img[ c["y"] - c_radius : c["y"] + c_radius, c["x"] - c_radius : c["x"] + c_radius ]
    if (i % 2) == 0: # even # robot #
      robot_color_averages.append(np.average(np.average(crop, axis=0), axis=0))
    else: # odd # player #
      player_color_averages.append(np.average(np.average(crop, axis=0), axis=0))

player_color = np.average(player_color_averages, axis=0)
robot_color = np.average(robot_color_averages, axis=0)

print("player: " + str(player_color))
print("robot: " + str(robot_color))

calib_data["robot"] = {"b": int(robot_color[0]),"g": int(robot_color[1]),"r": int(robot_color[2])}
calib_data["player"] = {"b": int(player_color[0]),"g": int(player_color[1]),"r": int(player_color[2])}

output = dump(calib_data, Dumper=Dumper)

outFile = open("coins.yaml", 'w')
outFile.write(output)
outFile.close()
#cap = cv.VideoCapture(0)
#
#if not cap.isOpened():
#    print("Cannot open camera")
#    exit()
#
#while True:
#    # Capture frame-by-frame
#    ret, frame = cap.read()
#    # if frame is read correctly ret is True
#    if not ret:
#        print("Can't receive frame (stream end?). Exiting ...")
#        break
#
#    # Display the resulting frame
#    cv.imshow('frame', frame)
#    if cv.waitKey(1) == ord('q'):
#        cv.imwrite("/home/davesarmoury/coins.jpg", frame)
#        break
## When everything done, release the capture
#cap.release()
#cv.destroyAllWindows()