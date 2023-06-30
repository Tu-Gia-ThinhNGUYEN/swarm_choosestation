#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
import tf
import random
import numpy as np
from jsk_rviz_plugins.msg import Pictogram, PictogramArray
from random import random, choice
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from swarm_choosestation.msg import Publishpoint
import csv

rospy.init_node('save_csv')

rospy.loginfo("#############Descriptions################")
rospy.loginfo("Use publish point tool to choose needing stations")
rospy.loginfo("***Press Ctrl+C to finish***")
rospy.loginfo("#########################################")

rate = rospy.Rate(1)  # Publish at a rate of 1 Hz
x = []
y = []
numberOfStations = 0
package_path = '/home/swarmpc/catkin_ws/src/swarm_choosestation/csv/pos.csv'

def callback(msg):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "/map"
    point.point.x = msg.point.x         #access to point data structure and its x, y, z components
    point.point.y = msg.point.y
    point.point.z = msg.point.z
    rospy.loginfo("coordinates:x=%f y=%f" %(point.point.x, point.point.y))
    global x, y
    x.append(point.point.x)
    y.append(point.point.y)

    # csv header
    fieldnames = ['x', 'y', 'z']
    # csv data
    rows = []
    for i in range(0,len(x)):
        rows.append({'x': x[i], 'y': y[i], 'z': 0})
    with open(package_path, 'w') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

def listener():
    rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)
    rospy.spin()

while not rospy.is_shutdown():
    listener()
    print("Data saved to ", package_path)