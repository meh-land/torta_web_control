#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Int16MultiArray

Pmax = 105
Pmin = 65
Rmax = 170
Rmin = 10

def constraint_prismatic(val):
    if val > Pmax:
    	return Pmax
    elif val < Pmin:
    	return Pmin
    else:
    	return val

def constraint_revolute(val):
    if val > Rmax:
    	return Rmax
    elif val < Rmin:
    	return Rmin
    else:
    	return val
    	
# Arm publisher
def arm_pub():
    pub = rospy.Publisher("arm_joints", Int16MultiArray, queue_size=10)
    rospy.init_node("arm_pub", anonymous=True)

    # Get arm values from arguments
    try:
        arm_val1 = constraint_prismatic(int(sys.argv[1]))
    except:
        arm_val1 = Pmin
    try:
        arm_val2 = constraint_revolute(int(sys.argv[2]))
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
