#!/bin/python3
# NOT TESTED
from jsonToGraph import Graph
import jsonToTask as jt
import os
from dotenv import load_dotenv
import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import numpy as np
from time import sleep

# Init variables
curr_x = None
curr_y = None
curr_theta = None

# Aux variables
curr_pose_vec = np.zeros(2)
target_pose_vec = np.zeros(2)
vel_vec = np.zeros(2)

def write_logs(log_str):
    global logs_file
    with open(logs_file, 'a') as file:
        file.write(log_str + "\n")

def vel_pub(vel_x, vel_y, vel_theta=0):
    vel_msg = Twist()
    vel_msg.linear.x = vel_x
    vel_msg.linear.y = vel_y
    vel_msg.angular.z = vel_theta
    pub.publish(vel_msg)
    vel_str = f"x = {vel_msg.linear.x}, y = {vel_msg.linear.y}, theta = {vel_msg.angular.z}"
    rospy.loginfo(vel_str)

def odom_callback(data):
    global curr_x, curr_y
    pos = data.pose.pose.position
    curr_x = pos.x
    curr_y = pos.y
    curr_pose_vec[0] = curr_x 
    curr_pose_vec[1] = curr_y 
    # rospy.loginfo("x: {}, y: {}".format(pos.x,pos.y))
    
def mpu_callback(data):
    global curr_theta
    curr_theta = data.data
    # rospy.loginfo("theta: {}".format(data.data))
    
# Init ros
rospy.init_node("task_handler", anonymous=True)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
arm_pub = rospy.Publisher("cmd_arm", Int16, queue_size=10)
rospy.Subscriber("odom", Odometry, odom_callback)
rospy.Subscriber("Mpu", Odometry, mpu_callback)

# Get task name from command line arguments
task_name = sys.argv[1]

# Load environment variables
load_dotenv()

# Access the variables
map_dir = os.getenv("MAPS_DIR")
matrix_dir = os.getenv("MATRIX_DIR")
tasks_dir = os.getenv("TASKS_DIR")
logs_dir = os.getenv("LOGS_DIR")
logs_file = logs_dir + "Logs.txt"

# Get task data
task_file_name = task_name.strip() + ".json"
task_file_path = tasks_dir + task_file_name
task = jt.readTask(task_file_path)

# Get map data
map_file_name = task.map.strip() + ".json"
map_file_path = map_dir + map_file_name
graph = Graph(map_file_path)

# Get optimal path (path is a list of nodes)
path = graph.get_path(task.pickupNode.strip(), task.dropoffNode.strip()).nodes
# Task nodes are defined by their labels, but we can benefit from defining them as node objects
task.pickupNode = path[0]
task.dropoffNode = path[-1]

while curr_x is None:
    # rospy.spin()
    continue

# Get my current location
curr_pose = [curr_x, curr_y, curr_theta]
min_dist = None
if not graph.origin_node.atNode(curr_pose):
    # This means that we are not at the origin node, so me need to find the closest node to our location
    for n in graph.nodes:
        if min_dist is None or n.distToNode(curr_pose)<min_dist:
            min_dist = n.distToNode(curr_pose)
            curr_node = n
    # If the loop ends and I am not at any node, then I am between two nodes, I will ignore this case for now, but maybe come back to it later
    #TODO: handle when between nodes
else:
    curr_node = graph.origin_node

curr_node = graph.origin_node
# If my current node is not the pickup node, go to the pickup node
#if not curr_node == task.pickupNode:
#    path_to_pickup = graph.get_path(curr_node.label, task.pickupNode.label)
#    path = path_to_pickup + path[1:]

def goToNode(n):
    # target_x, target_y, target_theta = n
    rospy.loginfo(f"current_odom = x: {curr_x}, y: {curr_y}")
    rospy.loginfo(f"Target Node = x: {n.X}, y: {n.Y}")
    target_x = n.X
    target_y = n.Y
    target_pose_vec[0] = n.X
    target_pose_vec[1] = n.Y
    curr_pose_vec[0] = curr_x
    curr_pose_vec[1] = curr_y
    vel_vec = target_pose_vec - curr_pose_vec
    vel_vec = 5 * vel_vec / (np.linalg.norm(vel_vec) + 1e-10)

    # Move in target direction
    vel_pub(vel_vec[0], vel_vec[1])
    
    # Wait till in range of 1 cm of target
    while(np.linalg.norm(target_pose_vec - curr_pose_vec) > 10e-2):
        # rospy.spin()
        continue

    # Stop
    vel_pub(0,0)


def pickupLoad():
    rospy.loginfo("picking up")
    write_logs("picking up")
    arm_pub.publish(1)
    sleep(2)

def dropoffLoad():
    rospy.loginfo("dropping off")
    write_logs("dropping off")
    arm_pub.publish(0)
    sleep(2)


# Start walking of the path
write_logs(f"Starting Task: {task_name}")
for n in path:
    goToNode(n)
    curr_node = n
    if curr_node == task.pickupNode:
        pickupLoad()
    elif curr_node == task.dropoffNode:
       dropoffLoad() 
write_logs(f"End Task: {task_name}")
