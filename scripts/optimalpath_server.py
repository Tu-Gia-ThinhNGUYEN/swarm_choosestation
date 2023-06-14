#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import json

rospy.init_node('optimalPath_server')

linestrip_pub = rospy.Publisher("/optimalpath_linestrip", Marker,  queue_size=10)

rate = rospy.Rate(1)  # Publish at a rate of 1 Hz
path = []
path_old = []
numberOfStations = 0
firstime_run = False
width = 384
originx = -9.6
originy = -9.6
resolution = 0.05

path_package = '/home/swarm/catkin_ws/src/swarm_pathfinding'
path_distanceMatrix = path_package+'/json/distanceMatrix.json'
path_pathTemp = path_package + '/json/pathTemp.json'
path_pathArray = path_package + '/json/pathArray.json'
path_pathComponent = path_package + '/json/pathComponent.json'
path_opttimalPath = path_package + '/json/opttimalPath.json'

def callback(path):  
    rospy.loginfo(path)  
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.action = marker.DELETEALL
    linestrip_pub.publish(marker)

    linestrip = Marker()
    linestrip.header.frame_id = 'map'
    linestrip.id = 1000
    linestrip.type = Marker.LINE_STRIP

    linestrip.scale.x = 0.02
    linestrip.color.a = 1.0
    linestrip.color.r = 0.0
    linestrip.color.g = 0.0
    linestrip.color.b = 0.0

    for index in range(0,len(path)):
        p = Point()
        p.x = ((path[index]%width)-abs(originx/resolution))*resolution
        p.y = ((int(path[index]/width))-abs(originy/resolution))*resolution
        p.z = 0.0

        linestrip.points.append(p)
    # Publish the MarkerArray message
    linestrip_pub.publish(linestrip)

rospy.loginfo("Optimal Path server is starting...")
# Opening JSON file
with open(path_opttimalPath) as openfile:
	# Reading from json file
	json_object = json.load(openfile)
path = json_object["p"]
path_old = path
rospy.loginfo("Wait for load path data file...")
rospy.sleep(1)

def listener():
    global firstime_run, path_old
    if not firstime_run:
        callback(path_old)
        rospy.sleep(1)
        firstime_run = True
    # Opening JSON file
    with open(path_opttimalPath) as openfile:
        # Reading from json file
        json_object = json.load(openfile)
        path = json_object["p"]
        if path != path_old:
            path_old = path
            callback(path)
    rospy.sleep(1)

while not rospy.is_shutdown():
    listener()