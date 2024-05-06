#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys

def talker():
    pub = rospy.Publisher("cmd_button", String, queue_size=10)
    rospy.init_node("button_pub", anonymous=True)
#    rate = rospy.Rate(10) # 10hz
    c = 0
    my_arg = sys.argv[1]
    rospy.loginfo(my_arg)
    pub.publish(my_arg)
#    while not rospy.is_shutdown():
#        c += 1
#        my_msg = my_arg + str(c)
#        rospy.loginfo(my_msg)
#        pub.publish(my_msg)
#        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
