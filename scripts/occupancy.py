#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid,MapMetaData
import sys
import math
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

curr_map = []

def callback(data):
    global curr_map
    metadata = data.info
    linear_data = np.array(data.data).reshape(-1,1)
    occupancy_map = np.reshape(linear_data,(metadata.height,metadata.width))
    curr_map = occupancy_map


def talker():
    
    # ack_pub = rospy.Publisher('/car_1/command', AckermannDrive, queue_size=10)
    rospy.init_node('occupancy_node', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(curr_map) !=0:
            plt.imshow(curr_map)
            plt.show()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass