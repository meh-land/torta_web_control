#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

begin_x=None
begin_y=None
def callback(data):
    global begin_x,begin_y
    pos = data.pose.pose.position
    if begin_x is None:
        begin_x=pos.x
        begin_y=pos.y
    rospy.loginfo("x: {}, y: {}".format(pos.x-begin_x,pos.y-begin_y))
    
def listener():


    rospy.init_node("odom_sub", anonymous=True)

    rospy.Subscriber("odom", Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
