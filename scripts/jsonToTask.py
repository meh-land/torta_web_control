#!/bin/python3
import json

class Task:
    def __init__(self, task_dict):
        # Attributes are not clear yet
        self.id = task_dict["id"]
        self.name = task_dict["name"]
        self.map = task_dict["map"]
        self.pickupNode = task_dict["pickupNode"]
        self.dropoffNode = task_dict["dropoffNode"]
        self.time = task_dict["taskTime"]

    def __repr__(self):
        # TODO
        repr_str = f"id = {self.id},name = {self.name}, map = {self.map}, pickupNode = {self.pickupNode}, dropoffNode = {self.dropoffNode}, time = {self.time}"
        return repr_str

def readTask(file_path):
    with open(file_path, 'r') as file:
        data_dict = json.load(file)
    return Task(data_dict)
