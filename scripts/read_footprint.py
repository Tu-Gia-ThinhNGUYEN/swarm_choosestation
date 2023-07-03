#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import json
from datetime import datetime
import math

filename = "20230615-204934_OKELA"
json_path = "/home/swarm/catkin_ws/src/swarm_choosestation/json/"+filename+".json"

linestrip1_pub = rospy.Publisher("/sw1_linestrip", Marker,  queue_size=10)
linestrip2_pub = rospy.Publisher("/sw2_linestrip", Marker,  queue_size=10)
linestrip3_pub = rospy.Publisher("/sw3_linestrip", Marker,  queue_size=10)

sw1_x = []
sw1_y = []

sw2_x = []
sw2_y = []

sw3_x = []
sw3_y = []

firsttime1 = False
firsttime2 = False
firsttime3 = False

xp = 0.0 # x position in grid coordinate
yp = 0.0 # y position in grid coordinate
xm = 0.0 # x position in meter coordinate
ym = 0.0 # y position in meter coordinate
originX = 960
originY = 960
resolution = 0.005 # 5cm per pixel
width = 3840

offsetPath = 70

def linestrip(x,y,id,r,g,b,linestrip_pub): 
    # print(x)
    # print(y)  
    linestrip = Marker()
    linestrip.header.frame_id = 'map'
    linestrip.id = id
    linestrip.type = Marker.LINE_STRIP

    linestrip.scale.x = 0.02
    linestrip.color.a = 1.0
    linestrip.color.r = r
    linestrip.color.g = g
    linestrip.color.b = b

    for index in range(0,len(x)-offsetPath):
        p = Point()
        p.x = (x[index]-originX)*resolution
        p.y = (y[index]-originY)*resolution
        p.z = 0.0
        linestrip.points.append(p)
    # Publish the MarkerArray message
    # rospy.loginfo("x: " + str((x[index]-originX)*resolution) + "     y: "+str((y[index]-originY)*resolution))
    linestrip_pub.publish(linestrip)  
    


with open(json_path, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)
    sw1_x = json_object["sw1_x"]
    sw1_y = json_object["sw1_y"]
    sw2_x = json_object["sw2_x"]
    sw2_y = json_object["sw2_y"]
    sw3_x = json_object["sw3_x"]
    sw3_y = json_object["sw3_y"]

def calculate_distance(x,y): 
    distance = 0.0
    if x is not None and y is not None:
        for index in range(0,len(x)-1-offsetPath):
            x1 = (x[index]-originX)*resolution
            y1 = (y[index]-originY)*resolution
            x2 = (x[index+1]-originX)*resolution
            y2 = (y[index+1]-originY)*resolution
            distance = distance + math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return distance


if __name__ == "__main__":
    rospy.init_node('footprint_server', anonymous=True) #make node
    distance_sw1 = calculate_distance(sw1_x,sw1_y)
    rospy.loginfo("Distance robot 1: " + str(distance_sw1) + "m")
    distance_sw2 = calculate_distance(sw2_x,sw2_y)
    rospy.loginfo("Distance robot 2: " + str(distance_sw2) + "m")
    distance_sw3 = calculate_distance(sw3_x,sw3_y)
    rospy.loginfo("Distance robot 3: " + str(distance_sw3) + "m")

    while not rospy.is_shutdown():
        rate = rospy.Rate(100) 
        linestrip(sw1_x,sw1_y,2001,1,0,0,linestrip1_pub)
        linestrip(sw2_x,sw2_y,2002,0,1,0,linestrip2_pub)
        linestrip(sw3_x,sw3_y,2003,0,0,1,linestrip3_pub)
        rate.sleep()
    rospy.spin()