import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# Global variables to store the Speed and received speed
Speed = Twist()
received_speed = Twist()
old_Speed = Twist()
IS_STOPPED = False
vel_pub = None

def need_stop(us_reading):
    return (us_reading < 10) and (us_reading > 0.1) and not IS_STOPPED

# Callback function for processing ultrasonic sensor data
def callback_position(Distances):
    global Speed, received_speed, old_Speed, IS_STOPPED
    # Check if any distance is less than 10
    if need_stop(Distances.data[0]) or need_stop(Distances.data[1]) or need_stop(Distances.data[2]) or need_stop(Distances.data[3]):
        # Stop the robot by setting linear and angular velocities to 0
        old_Speed = received_speed
        Speed.linear.x = 0.0
        Speed.linear.y = 0.0
        Speed.angular.z = 0.0
        vel_pub.publish(Speed)
        IS_STOPPED = True
        rospy.loginfo("stopped")
    else:
        # If distances are safe, use the received speed values
        if IS_STOPPED:
            IS_STOPPED = False
            vel_pub.publish(old_Speed)
            rospy.loginfo("resume motion")

# Callback function for processing cmd_vel data
def callback_speed(speed_msg):
    global received_speed
    # Store the received speed values
    if not IS_STOPPED:
        received_speed = speed_msg

def main():
    global vel_pub

    # Initialize the ROS node
    rospy.init_node('Stop_Crash', anonymous=True)

    # Publisher to publish speed commands
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)

    # Subscriber to receive ultrasonic sensor data
    rospy.Subscriber('Ultrasonics', Float32MultiArray, callback_position)

    # Subscriber to receive speed commands
    rospy.Subscriber('cmd_vel', Twist, callback_speed)

    # Setting the loop rate (frequency)
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # Sleep to maintain the loop rate
        rate.sleep()

        # Process callbacks
        rospy.spinOnce()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
