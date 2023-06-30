#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import json
from datetime import datetime

filename = datetime.now().strftime("%Y%m%d-%H%M%S")
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

def linestrip(x,y,id,r,g,b,linestrip_pub):   
    linestrip = Marker()
    linestrip.header.frame_id = 'map'
    linestrip.id = id
    linestrip.type = Marker.LINE_STRIP

    linestrip.scale.x = 0.02
    linestrip.color.a = 1.0
    linestrip.color.r = r
    linestrip.color.g = g
    linestrip.color.b = b

    for index in range(0,len(x)):
        p = Point()
        p.x = (x[index]-originX)*resolution
        p.y = (y[index]-originY)*resolution
        p.z = 0.0
        linestrip.points.append(p)
    # Publish the MarkerArray message
    rospy.loginfo("x: " + str((x[index]-originX)*resolution) + "     y: "+str((y[index]-originY)*resolution))
    linestrip_pub.publish(linestrip)

def odometrysw1(msg):
    global sw1_x, sw1_y, firsttime1
    rate = rospy.Rate(100) # 10hz
    if not firsttime1:
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        xp = originX + int(xm/resolution)
        yp = originY + int(ym/resolution)
        sw1_x.append(xp)
        sw1_y.append(yp)
        firsttime1 = True
    else:
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        xp = originX + int(xm/resolution)
        yp = originY + int(ym/resolution)
        # print(str(sw1_x) +"     "+str(sw1_y))
        if (sw1_x[-1] != xp) or (sw1_y[-1] != yp):
            sw1_x.append(xp)
            sw1_y.append(yp)
            linestrip(sw1_x,sw1_y,2001,1,0,0,linestrip1_pub)
            rospy.loginfo("xm: " + str(xm) + "     ym: "+str(ym))
        rate.sleep()

def odometrysw2(msg):
    global sw2_x, sw2_y, firsttime2
    rate = rospy.Rate(100) # 10hz
    if not firsttime2:
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        xp = originX + int(xm/resolution)
        yp = originY + int(ym/resolution)
        sw2_x.append(xp)
        sw2_y.append(yp)
        firsttime2 = True
    else:
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        xp = originX + int(xm/resolution)
        yp = originY + int(ym/resolution)
        # print(str(sw1_x) +"     "+str(sw1_y))
        if (sw2_x[-1] != xp) or (sw2_y[-1] != yp):
            sw2_x.append(xp)
            sw2_y.append(yp)
            linestrip(sw2_x,sw2_y,2002,0,1,0,linestrip2_pub)
        rate.sleep()
def odometrysw3(msg):
    global sw3_x, sw3_y, firsttime3
    rate = rospy.Rate(100) # 10hz
    if not firsttime3:
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        xp = originX + int(xm/resolution)
        yp = originY + int(ym/resolution)
        sw3_x.append(xp)
        sw3_y.append(yp)
        firsttime3 = True
    else:
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        xp = originX + int(xm/resolution)
        yp = originY + int(ym/resolution)
        # print(str(sw1_x) +"     "+str(sw1_y))
        if (sw3_x[-1] != xp) or (sw3_y[-1] != yp):
            sw3_x.append(xp)
            sw3_y.append(yp)
            linestrip(sw3_x,sw3_y,2003,0,0,1,linestrip3_pub)
        rate.sleep()       

if __name__ == "__main__":
    rospy.init_node('footprint_server', anonymous=True) #make node
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,odometrysw1)
    rospy.Subscriber('/sw2/amcl_pose',PoseWithCovarianceStamped,odometrysw2)
    rospy.Subscriber('/sw3/amcl_pose',PoseWithCovarianceStamped,odometrysw3)
    rospy.spin()
    rospy.loginfo("Path has been saved at: "+json_path)
    dictionary = {
        "sw1_x": sw1_x,
        "sw1_y": sw1_y,
        "sw2_x": sw2_x,
        "sw2_y": sw2_y,
        "sw3_x": sw3_x,
        "sw3_y": sw3_y
    }
    json_object = json.dumps(dictionary, indent=6)
    with open(json_path, "w") as outfile:
	    outfile.write(json_object)