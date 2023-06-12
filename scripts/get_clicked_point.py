#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
import csv

filename = "/home/swarm/catkin_ws/src/swarm_choosestation/csv/pos.csv"  # Specify the filename for the CSV file
x = 0.0
y = 0.0
z = 0.0

rospy.loginfo("#################################################")
rospy.loginfo("Use publish point tool to choose needing stations")
rospy.loginfo("#################################################")

def callback(msg):
    global x, y
    x = msg.point.x
    y = msg.point.y
    rospy.loginfo("coordinates: x=%f y=%f" % (x, y))
    save_to_csv(filename)

def listener():
    rospy.init_node('goal_publisher', anonymous=True)
    rospy.Subscriber('/clicked_point', PointStamped, callback)
    rospy.spin()

def save_to_csv(filename):
    data = [[x, y, z]]
    # Get the package path
    with open(filename, 'w') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerows(data)
    

if __name__ == '__main__':
    listener()
    print("Data saved to", filename)
    
