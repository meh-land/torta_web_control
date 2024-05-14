#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Int16MultiArray

# PID publisher
def arm_pub():
    pub = rospy.Publisher("arm_joints", Int16MultiArray, queue_size=10)
    rospy.init_node("arm_pub", anonymous=True)

    # Get arm values from arguments
    try:
        arm_val1 = int(sys.argv[1])
    except:
        arm_val1 = 0
    try:
        arm_val2 = int(sys.argv[2])
    except:
        arm_val2 = 0
    # Create an Int16MultiArray message
    arm_msg = Int16MultiArray()

    # Fill the data array with some example values
    arm_msg.data = [arm_val1, arm_val2]

    rospy.loginfo(arm_msg)
    pub.publish(arm_msg)

if __name__ == '__main__':
    try:
        arm_pub()
    except rospy.ROSInterruptException:
        pass
