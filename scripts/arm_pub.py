#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Float32

# PID publisher
def arm_pub():
    pub = rospy.Publisher("cmd_arm", Float32, queue_size=10)
    rospy.init_node("arm_pub", anonymous=True)

    # Get arm values from arguments
    try:
        arm_val = float(sys.argv[1])
    except:
        arm_val = 0
   
    # Publish recieved values
    arm_str = str(arm_val)

    rospy.loginfo(arm_str)
    pub.publish(arm_val)

if __name__ == '__main__':
    try:
        arm_pub()
    except rospy.ROSInterruptException:
        pass
