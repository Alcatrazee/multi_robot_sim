#!/usr/bin/env python
import rospy
from cv2 import *
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid


def server_callback(req):

    print('request received!')
    img = imread(
        "/home/alcatraz/catkin_ws/src/multiple_rb_ctrl/maps/map.pgm", 0)
    eroded = erode(img, np.ones((5, 5), np.uint8), iterations=1)
    imwrite("/home/alcatraz/catkin_ws/src/multiple_rb_ctrl/maps/eroded_map.pgm", eroded)
    resp = OccupancyGrid()
    resp.header.frame_id = "map"
    resp.info.resolution = 0.05
    resp.info.width = 4000
    resp.info.height = 4000
    i = 0
    print("processing map.")
    for row in range(len(eroded)):
        for column in range(len(eroded[0])):
            resp.data.append((eroded[row][column]*0.390625))
            i += 1
    print("sending back.")
    return resp


if __name__ == "__main__":
    rospy.init_node("cost_map_node")
    server = rospy.Service("cost_map_server", GetMap, server_callback)
    print("cost map node running")
    while not rospy.is_shutdown():
        rospy.spin()
