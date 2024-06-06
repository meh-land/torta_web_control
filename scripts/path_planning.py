#!/bin/python3
from jsonToGraph import Graph
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Access the variables
map_dir = os.getenv("MAPS_DIR")
matrix_dir = os.getenv("MATRIX_DIR")

file_name = "myMap.json"
file_path = map_dir + file_name
graph = Graph(file_path)
print(graph.get_path('Entrance', 'node100').nodes)

