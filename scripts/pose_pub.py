#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Pose

# Position publisher
def pose_pub():
    pub = rospy.Publisher("cmd_pose", Pose, queue_size=10)
    rospy.init_node("pose_pub", anonymous=True)

    # Get Pose values from arguments
    try:
        pose_x = float(sys.argv[1])
    except:
        pose_x = 0
    try:
        pose_y = float(sys.argv[2])
    except:
        pose_y = 0
    try:
        pose_theta = float(sys.argv[3])
    except:
        pose_theta = 0
    
    # Put values in pose msg
    pose_msg = Pose()
    # Publish recieved values
    pose_msg.position.x = pose_x
    pose_msg.position.y = pose_y
    pose_msg.orientation.w = pose_theta
 
    pose_str = f"x = {pose_msg.position.x}, y = {pose_msg.position.y}, theta = {pose_msg.orientation.w}"

    rospy.loginfo(pose_str)
    pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        pose_pub()
    except rospy.ROSInterruptException:
        pass
