#!/bin/python3
# NOT TESTED
from jsonToGraph import Graph
import jsonToTask as jt
import os
from dotenv import load_dotenv
import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from time import sleep

# Init variables
curr_x = None
curr_y = None
curr_theta = None


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
    rospy.loginfo("x: {}, y: {}".format(pos.x,pos.y))
    
def mpu_callback(data):
    global curr_theta
    curr_theta = data.data
    rospy.loginfo("theta: {}".format(data.data))
    
# Init ros
rospy.init_node("task_handler", anonymous=True)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
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
    rospy.spinOnce()

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

# If my current node is not the pickup node, go to the pickup node
if not curr_node == task.pickupNode:
    path_to_pickup = graph.get_path(curr_node.label, task.pickupNode.label)
    path = path_to_pickup + path[1:]

def goToNode(n):
    target_x, target_y, target_theta = n
    delta_x = (target_x-curr_x)
    delta_y = (target_y-curr_y)

    # Move in direction of target x
    vel_pub(delta_x/abs(delta_x)*5,0)

    # Wait till in range of 10 cm of target
    while(abs(curr_x-target_x)<0.1):
        rospy.spinOnce()
        continue

    # Move in direction of target y
    vel_pub(0,delta_y/abs(delta_y)*5)

    # Wait till 10 cm of target
    while(abs(curr_y-target_y)>0.1):
        rospy.spinOnce()

    # Stop
    vel_pub(0,0)


def pickupLoad():
    #TODO
    sleep(10)

def dropoffLoad():
    #TODO
    sleep(10)


# Start walking of the path
for n in path:
    goToNode(n)
    if curr_node == task.pickupNode:
        pickupLoad()
    elif curr_node == task.dropoffNode:
       dropoffLoad() 

