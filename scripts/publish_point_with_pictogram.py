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
# from swarm_choose_stations.msg import Publishpoint

rospy.init_node('marker_array_publisher')

ur_location = destination = Point()

pictogram_pub = rospy.Publisher("/pictogram_array", PictogramArray,  queue_size=10)
marker_array_pub = rospy.Publisher('marker_array_topic', MarkerArray, queue_size=10)
marker_text_pub = rospy.Publisher('marker_text_topic', MarkerArray, queue_size=10)
# publishpoints_pub = rospy.Publisher('publishpoint', Publishpoint, queue_size=10)

rate = rospy.Rate(100)  # Publish at a rate of 1 Hz
pictograms = []
x = []
y = []
numberOfStations = 0

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

    global numberOfStations
    numberOfStations = numberOfStations + 1

    # Create a MarkerArray message
    marker_array = MarkerArray()
    for index in range(0,len(x)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = numberOfStations
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x[index]
        marker.pose.position.y = y[index]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        # Append markers to the MarkerArray
        marker_array.markers.append(marker)
    # Publish the MarkerArray message
    marker_array_pub.publish(marker_array)

    # Create a Text message
    marker_text = MarkerArray()
    for index in range(0,len(x)):
        text = Marker()
        text.header.frame_id = "map"
        text.id = numberOfStations+100
        text.type = text.TEXT_VIEW_FACING
        text.action = text.ADD
        text.pose.position.x = x[index]
        text.pose.position.y = y[index]
        text.pose.position.z = 1.0
        # text.pose.orientation.w = 1.0
        text.scale.x = 0.3
        text.scale.y = 0.3
        text.scale.z = 0.3
        text.color.a = 1.0
        text.color.r = 255.0/255.0
        text.color.g = 0.0/255.0
        text.color.b = 0.0/255.0
        text.text = "Station " + str(numberOfStations)
        # Append markers to the MarkerArray
        marker_text.markers.append(text)
    # Publish the MarkerArray message
    marker_text_pub.publish(marker_text)

    # Create a Pictogram message
    arr = PictogramArray()
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
        msg.size = 1
        msg.color.r = 255 / 255.0
        msg.color.g = 0 / 255.0
        msg.color.b = 0 / 255.0
        msg.color.a = 1.0
        msg.character = "fa-arrow-down"
        arr.pictograms.append(msg)
    pictogram_pub.publish(arr)

    # pubpoint_msg = Publishpoint()
    # pubpoint_msg.message = "Stations are: "
    # pubpoint_msg.x = x
    # pubpoint_msg.y = y
    # publishpoints_pub.publish(pubpoint_msg)

    pub = rospy.Publisher('publish_point', Publishpoint, queue_size=10)
    msg = Publishpoint()
    msg.posx = x
    msg.posy = y
    rospy.loginfo(msg)
    pub.publish(msg)

    # rate.sleep()
    # rospy.spin()

def listener():
    rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)
    rospy.spin()

while not rospy.is_shutdown():
    listener()