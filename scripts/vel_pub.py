#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Twist

# Velocity publisher
def vel_pub():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.init_node("vel_pub", anonymous=True)

    # Get Vel values from arguments
    try:
        vel_x = float(sys.argv[1])
    except:
        vel_x = 0
    try:
        vel_y = float(sys.argv[2])
    except:
        vel_y = 0
    try:
        vel_theta = float(sys.argv[3])
    except:
        vel_theta = 0
    
    # Put values in twist msg
    vel_msg = Twist()
    # Publish recieved values
    vel_msg.linear.x = vel_x
    vel_msg.linear.y = vel_y
    vel_msg.angular.z = vel_theta
 
    vel_str = f"x = {vel_msg.linear.x}, y = {vel_msg.linear.y}, theta = {vel_msg.angular.z}"

    rospy.loginfo(vel_str)
    pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        vel_pub()
    except rospy.ROSInterruptException:
        pass
