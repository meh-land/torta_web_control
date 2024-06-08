#!/bin/python3
# NOT TESTED
from jsonToGraph import Graph
import jsonToTask as jt
import os
from dotenv import load_dotenv
import sys
import rospy
from geometry_msgs.msg import Pose
import numpy as np

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

# Get my current location
curr_pose = np.loadtxt(logs_dir + "pose.log")
# pose.log should contain 0 0 0 at the beginning if it doesn't this means that this is not the firs run and we need to read the last row of data as our true curr_pose
if curr_pose.size > 3:
    curr_pose = curr_pose[-1]

if not graph.origin_node.atNode(curr_pose):
    # This means that we are not at the origin node, so me need to find the closest node to our location
    for n in graph.nodes:
        if n.atNode(curr_pose):
            curr_node = n
            break
    # If the loop ends and I am not at any node, then I am between two nodes, I will ignore this case for now, but maybe come back to it later
    #TODO: handle when between nodes
else:
    curr_node = graph.origin_node

# If my current node is not the pickup node, go to the pickup node
if not curr_node == task.pickupNode:
    path_to_pickup = graph.get_path(curr_node.label, task.pickupNode.label)
    path = path_to_pickup + path[1:]



