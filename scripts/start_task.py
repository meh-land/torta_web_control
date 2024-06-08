#!/bin/python3
# NOT TESTED
from jsonToGraph import Graph
import jsonToTask as jt
import os
from dotenv import load_dotenv
import sys
import rospy
from std_msgs.msg import String

# Get task name from command line arguments
task_name = sys.argv[1]

# Load environment variables
load_dotenv()

# Access the variables
map_dir = os.getenv("MAPS_DIR")
matrix_dir = os.getenv("MATRIX_DIR")
tasks_dir = os.getenv("TASKS_DIR")

# Get task data
task_file_name = task_name.strip() + ".json"
task_file_path = tasks_dir + task_file_name
task = jt.readTask(task_file_path)

# Get map data
map_file_name = task.map.strip() + ".json"
map_file_path = map_dir + map_file_name
graph = Graph(map_file_path)
path = graph.get_path(task.pickupNode.strip(), task.dropoffNode.strip()).nodes


def talker():
    # Initialize the ROS node with the name 'talker'
    rospy.init_node('talker', anonymous=True)
    # Create a Publisher object, publishing to the 'chatter' topic with String messages
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # Set the loop rate in Hz (e.g., 10 Hz means 10 messages per second)
    rate = rospy.Rate(1) # 10 Hz
    while not rospy.is_shutdown():
        # Create a string message with the current time
        hello_str = f"n1 = {path[0].label}, {path[-1].label}"
        # Log the message to the console
        rospy.loginfo(hello_str)
        # Publish the message to the 'chatter' topic
        pub.publish(hello_str)
        # Sleep for the remaining time to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


