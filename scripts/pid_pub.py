#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Point

# PID publisher
def pid_pub():
    pub = rospy.Publisher("cmd_pid", Point, queue_size=10)
    rospy.init_node("pid_pub", anonymous=True)

    # Get PID values from arguments
    try:
        kp = float(sys.argv[1])
    except:
        kp = 0
    try:
        ki = float(sys.argv[2])
    except:
        ki = 0
    try:
        kd = float(sys.argv[3])
    except:
        kd = 0
    
    # Put values in Point msg
    pid_msg = Point()
    # Publish recieved values
    pid_msg.x = kp 
    pid_msg.y = ki
    pid_msg.z = kd
 
    pid_str = f"x = {pid_msg.x}, y = {pid_msg.y}, theta = {pid_msg.z}"

    rospy.loginfo(pid_str)
    pub.publish(pid_msg)

if __name__ == '__main__':
    try:
        pid_pub()
    except rospy.ROSInterruptException:
        pass
