#!/bin/python3
# NOT TESTED
from jsonToGraph import Graph
import os
from dotenv import load_dotenv
import sys

# Get task name from command line arguments
task_name = sys.argv[1]

# Load environment variables
load_dotenv()

# Access the variables
map_dir = os.getenv("MAPS_DIR")
matrix_dir = os.getenv("MATRIX_DIR")
tasks_dir = os.getenv("TASKS_DIR")

# Get task data
map_file_name = "myMap.json"
map_file_path = map_dir + file_name

# Get map data
map_file_name = "myMap.json"
map_file_path = map_dir + file_name
graph = Graph(file_path)
print(graph.get_path('Entrance', 'node100').nodes)


