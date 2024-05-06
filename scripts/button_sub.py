#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(data.data)
    
def listener():


    rospy.init_node("button_sub", anonymous=True)

    rospy.Subscriber("cmd_button", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
