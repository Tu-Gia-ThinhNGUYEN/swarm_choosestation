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
# from swarm_choose_stations.msg import Publishpoint

rospy.init_node('pictogram_server')

ur_location = destination = Point()

pictogram_pub = rospy.Publisher("/pictogram_array", PictogramArray,  queue_size=10)
marker_array_pub = rospy.Publisher('marker_array_topic', MarkerArray, queue_size=10)
marker_text_pub = rospy.Publisher('marker_text_topic', MarkerArray, queue_size=10)
# publishpoints_pub = rospy.Publisher('publishpoint', Publishpoint, queue_size=10)

rate = rospy.Rate(1)  # Publish at a rate of 1 Hz
pictograms = []
fieldnames = ['x', 'y', 'z']
x = []
y = []
x_old = []
y_old = []
numberOfStations = 0
firstime_run = False
package_path = '/home/swarmpc/catkin_ws/src/swarm_choosestation/csv/pos.csv'

def callback(x,y): 
    global numberOfStations
    numberOfStations = len(x)
    rospy.loginfo('x: ' + str(x))
    rospy.loginfo('y: ' + str(y))

    # Create a MarkerArray message
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.action = marker.DELETEALL
    marker_array.markers.append(marker)
    marker_array_pub.publish(marker_array)

    for index in range(0,len(x)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x[index]
        marker.pose.position.y = y[index]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        # Append markers to the MarkerArray
        marker_array.markers.append(marker)
    # Publish the MarkerArray message
    marker_array_pub.publish(marker_array)

    # Create a Text message
    marker_text = MarkerArray()
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.action = marker.DELETEALL
    marker_text.markers.append(marker)
    marker_text_pub.publish(marker_text)

    for index in range(0,len(x)):
        text = Marker()
        text.header.frame_id = "map"
        text.id = index+100
        text.type = text.TEXT_VIEW_FACING
        text.action = text.ADD
        text.pose.position.x = x[index]
        text.pose.position.y = y[index]
        text.pose.position.z = 0.6
        # text.pose.orientation.w = 1.0
        text.scale.x = 0.2
        text.scale.y = 0.2
        text.scale.z = 0.2
        text.color.a = 0.5
        text.color.r = 0.0/255.0
        text.color.g = 0.0/255.0
        text.color.b = 0.0/255.0
        text.text = "Station " + str(index+1)
        # Append markers to the MarkerArray
        marker_text.markers.append(text)
    # Publish the MarkerArray message
    marker_text_pub.publish(marker_text)

    # Create a Pictogram message
    arr = PictogramArray()
    marker = Pictogram()
    marker.header.frame_id = 'map'
    marker.action = marker.DELETE
    arr.pictograms.append(marker)
    pictogram_pub.publish(arr)

    arr.header.frame_id = "/map"
    arr.header.stamp = rospy.Time.now()
    for index in range(0,len(x)):
        msg = Pictogram()
        msg.header.frame_id = "/map"
        msg.action = Pictogram.JUMP
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x[index]
        msg.pose.position.y = y[index]
        msg.pose.position.z = 0.1
        # It has to be like this to have them vertically orient the icons.
        msg.pose.orientation.w = 0.7
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = -0.7
        msg.pose.orientation.z = 0
        msg.size = 0.5
        msg.color.r = 0 / 255.0
        msg.color.g = 0 / 255.0
        msg.color.b = 0 / 255.0
        msg.color.a = 0.5
        msg.character = "fa-arrow-down"
        arr.pictograms.append(msg)
    pictogram_pub.publish(arr)

with open(package_path) as f:
    csv_reader = csv.DictReader(f, fieldnames)
    next(csv_reader)
    for line in csv_reader:
        x.extend({line['x']})
        y.extend({line['y']})
    # convert string list to float list
    x = [float(x) for x in x]
    y = [float(y) for y in y]
    x_old = x
    y_old = y
    # callback(x,y)
    rospy.sleep(1)

def listener():
    global x_old,y_old, firstime_run
    x = []
    y = []
    if not firstime_run:
        callback(x_old,y_old)
        rospy.sleep(1)
        firstime_run = True
    with open(package_path) as f:
        csv_reader = csv.DictReader(f, fieldnames)
        next(csv_reader)
        for line in csv_reader:
            x.extend({line['x']})
            y.extend({line['y']})
        # convert string list to float list
        x = [float(x) for x in x]
        y = [float(y) for y in y]
        if x_old != x or y_old != y:
            x_old = x
            y_old = y
            callback(x,y)
    # print(x_old)
    # print(x)
    rospy.sleep(1)

while not rospy.is_shutdown():
    listener()