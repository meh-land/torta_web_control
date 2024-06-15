#!/usr/bin/env python3
import os
from dotenv import load_dotenv
import rospy
from nav_msgs.msg import Odometry

# Load environment variables
load_dotenv()
logs_dir = os.getenv("LOGS_DIR")
logs_file = logs_dir + "Status.txt"

curr_x = 0
curr_y = 0
curr_theta = 0

def write_logs(log_str):
    global logs_file
    with open(logs_file, 'a') as file:
        file.write(log_str + "\n")


def odom_callback(data):
    global curr_x, curr_y
    pos = data.pose.pose.position
    prev_x = curr_x
    prev_y = curr_y
    curr_x = pos.x
    curr_y = pos.y
    if abs(curr_x - prev_x) > 1 or abs(curr_y - prev_y) > 1 or True:
        write_logs(f"{curr_x:.2f}, {curr_y:.2f}, {curr_theta:.2f}")
 
#def mpu_callback(data):
#    global curr_theta
#    curr_theta = data.data
 
# ROS stuff
rospy.init_node("logger", anonymous=True)
rospy.Subscriber("odom", Odometry, odom_callback)
#rospy.Subscriber("Mpu", Odometry, mpu_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
