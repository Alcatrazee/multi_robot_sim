#!/usr/bin/env python
import cv2
import numpy as np

if __name__ == "__main__":
    img = cv2.imread(
        "/home/alcatraz/catkin_ws/src/multiple_rb_ctrl/maps/map.pgm", 0)
    cropped = img[1800:2200, 1800:2200]
    cv2.imshow("cropped", cropped)
    cv2.imwrite("new_map.pgm", cropped)
    cv2.waitKey(0)
