#!/usr/bin/env python3
import rospy as rp
from geometry_msgs.msg import Twist
from time import sleep
from flask import Flask, render_template, request

# Initialize ros node
rp.init_node("serv_pub")
rp.loginfo("hello from serv_pub")
# Create publisher
pub = rp.Publisher("/cmd_vel", Twist, queue_size=10)


# Initialize flask
app = Flask(__name__)
@app.route("/", methods=["GET", "POST"])
def index():
    
    if request.method == "POST":
        x = request.form.get("x")
        y = request.form.get("y")
        theta = request.form.get("theta")
        try:
            x = float(x)
        except:
            x = 0

        try:
            y = float(y)
        except:
            y = 0
        try:
            theta = float(theta)
        except:
            theta = 0

        my_msg = Twist()
        # Publish recieved values
        my_msg.linear.x = x
        my_msg.linear.y = y
        my_msg.angular.z = theta
        msg_str = f"x = {my_msg.linear.x}, y = {my_msg.linear.y}, theta = {my_msg.angular.z}"
        pub.publish(my_msg)
        rp.loginfo(msg_str)
    return render_template("index.html")

    

