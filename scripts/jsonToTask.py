#!/bin/python3
import json

class Task:
    def __init__(self, task_dict):
        # Attributes are not clear yet
        self.id = task_dict["id"]
        self.label = task_dict["label"]
        self.map = task_dict["map"]
        self.pickup_node = task_dict["pickup_node"]
        self.dropoff_node = task_dict["dropoff_node"]
        self.time = task_dict["time"]

    def __repr__(self):
        # TODO
        ...

def readTask(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
