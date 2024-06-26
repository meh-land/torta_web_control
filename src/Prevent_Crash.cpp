#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

// Global variables to store the Speed and received speed
geometry_msgs::Twist Speed;
geometry_msgs::Twist received_speed;
geometry_msgs::Twist old_Speed;
bool IS_STOPPED = false;
ros::Publisher vel_pub;

bool need_stop(float us_reading){
	return (us_reading < 10) && (us_reading > 0.1);
}
// Callback function for processing ultrasonic sensor data
void Callback_Position(const std_msgs::Float32MultiArray::ConstPtr& Distances)
{
  // Check if any distance is less than 10
  if (need_stop(Distances->data[0]) || need_stop(Distances->data[1]) || need_stop(Distances->data[2]) || need_stop(Distances->data[3]))
  {
    // Stop the robot by setting linear and angular velocities to 0
    Speed.linear.x = 0.0;
    Speed.linear.y = 0.0;
    Speed.angular.z = 0.0;
    IS_STOPPED = true;
    old_Speed=received_speed;
    vel_pub.publish(Speed);
  }
  else
  {
    // If distances are safe, use the received speed values
    if(IS_STOPPED)
    {
    	IS_STOPPED = false;
	vel_pub.publish(old_Speed);
    }
  }
}

// Callback function for processing cmd_vel data
void Callback_Speed(const geometry_msgs::Twist::ConstPtr& speed_msg)
{
  // Store the received speed values
  received_speed = *speed_msg;
}

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "Stop_Crash");
  ros::NodeHandle nh;

  // Publisher to publish speed commands
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // Subscriber to receive ultrasonic sensor data
  ros::Subscriber ultrasonics_sub = nh.subscribe("Ultrasonics", 1000, Callback_Position);

  // Subscriber to receive speed commands
  ros::Subscriber speed_sub = nh.subscribe("cmd_vel", 1000, Callback_Speed);

  // Setting the loop rate (frequency)
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Publish the Speed message
    //vel_pub.publish(Speed);

    // Sleep to maintain the loop rate
    loop_rate.sleep();

    // Process callbacks
    ros::spinOnce();
  }

  return 0;
}

