#!/usr/bin/env python
from cv2 import *
import numpy as np

if __name__ == "__main__":
    img = imread(
        "/home/alcatraz/catkin_ws/src/multiple_rb_ctrl/maps/map.pgm", 0)
    eroded = erode(img, np.ones((19, 19), np.uint8), iterations=1)
    imwrite("/home/alcatraz/catkin_ws/src/multiple_rb_ctrl/maps/eroded_map.pgm", eroded)
