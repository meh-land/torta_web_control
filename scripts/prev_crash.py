#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

IS_STOPPED = False
front = 0
back = 1
right = 2
left = 3
curr_vel = None
old_vel = None
rospy.init_node("prev_crash", anonymous=True)
vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

def vel_pub_func(vel_x, vel_y, vel_theta=0):
    vel_msg = Twist()
    vel_msg.linear.x = vel_x
    vel_msg.linear.y = vel_y
    vel_msg.angular.z = vel_theta
    vel_pub.publish(vel_msg)


def need_stop(us_reading):
    return (us_reading < 10) and (us_reading > 0.1)


def us_callback(data):
    global IS_STOPPED, old_vel, curr_vel
    readings = data.data
    fns = need_stop(readings[front]) #front need stop
    bns = need_stop(readings[back])
    rns = need_stop(readings[right])
    lns = need_stop(readings[left])
    if (fns or bns or rns or lns) and not IS_STOPPED:
        old_vel = curr_vel
        vel_pub_func(0,0)
        IS_STOPPED = True
        rospy.loginfo("stopped")
    elif (fns or bns or rns or lns) and IS_STOPPED:
        return
    elif not (fns or bns or rns or lns) and IS_STOPPED:
        vel_pub.publish(old_vel)
        IS_STOPPED = False
        rospy.loginfo("resume")
    
def vel_callback(data):
    global curr_vel
    curr_vel = data




rospy.Subscriber("Ultrasonics", Float32MultiArray, us_callback)
rospy.Subscriber("cmd_vel", Twist, vel_callback)
if __name__ == '__main__':
    rospy.spin()
