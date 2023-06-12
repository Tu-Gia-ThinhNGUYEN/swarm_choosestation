#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

robotname1_pub = rospy.Publisher("/sw1_robotname", Marker,  queue_size=10)
robotname2_pub = rospy.Publisher("/sw2_robotname", Marker,  queue_size=10)
robotname3_pub = rospy.Publisher("/sw3_robotname", Marker,  queue_size=10)

xm = 0.0 # x position in meter coordinate
ym = 0.0 # y position in meter coordinate

def sw1robotname(msg):
    rate = rospy.Rate(100) # 10hz
    xm = msg.pose.pose.position.x
    ym = msg.pose.pose.position.y
    # Create a Text message
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.action = marker.DELETEALL
    robotname1_pub.publish(marker)
    text = Marker()
    text.header.frame_id = "map"
    text.id = 3000
    text.type = text.TEXT_VIEW_FACING
    text.action = text.ADD
    text.pose.position.x = xm
    text.pose.position.y = ym
    text.pose.position.z = 0.3
    # text.pose.orientation.w = 1.0
    text.scale.x = 0.15
    text.scale.y = 0.15
    text.scale.z = 0.15
    text.color.a = 1.0
    text.color.r = 255.0/255.0
    text.color.g = 0.0/255.0
    text.color.b = 0.0/255.0
    text.text = "Leader"
    # Publish the MarkerArray message
    robotname1_pub.publish(text)
    rate.sleep()

def sw2robotname(msg):
    rate = rospy.Rate(100) # 10hz
    xm = msg.pose.pose.position.x
    ym = msg.pose.pose.position.y
    # Create a Text message
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.action = marker.DELETEALL
    robotname2_pub.publish(marker)
    text = Marker()
    text.header.frame_id = "map"
    text.id = 3001
    text.type = text.TEXT_VIEW_FACING
    text.action = text.ADD
    text.pose.position.x = xm
    text.pose.position.y = ym
    text.pose.position.z = 0.3
    # text.pose.orientation.w = 1.0
    text.scale.x = 0.15
    text.scale.y = 0.15
    text.scale.z = 0.15
    text.color.a = 1.0
    text.color.r = 255.0/255.0
    text.color.g = 0.0/255.0
    text.color.b = 0.0/255.0
    text.text = "Follower 1"
    # Publish the MarkerArray message
    robotname2_pub.publish(text)
    rate.sleep()

def sw3robotname(msg):
    rate = rospy.Rate(100) # 10hz
    xm = msg.pose.pose.position.x
    ym = msg.pose.pose.position.y
    # Create a Text message
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.action = marker.DELETEALL
    robotname3_pub.publish(marker)
    text = Marker()
    text.header.frame_id = "map"
    text.id = 3002
    text.type = text.TEXT_VIEW_FACING
    text.action = text.ADD
    text.pose.position.x = xm
    text.pose.position.y = ym
    text.pose.position.z = 0.3
    # text.pose.orientation.w = 1.0
    text.scale.x = 0.15
    text.scale.y = 0.15
    text.scale.z = 0.15
    text.color.a = 1.0
    text.color.r = 255.0/255.0
    text.color.g = 0.0/255.0
    text.color.b = 0.0/255.0
    text.text = "Follower 2"
    # Publish the MarkerArray message
    robotname3_pub.publish(text)
    rate.sleep()

if __name__ == "__main__":
    rospy.init_node('robotname_server', anonymous=True) #make node
    rospy.Subscriber('/sw1/odom',Odometry,sw1robotname)
    rospy.Subscriber('/sw2/odom',Odometry,sw2robotname)
    rospy.Subscriber('/sw3/odom',Odometry,sw3robotname)
    rospy.spin()