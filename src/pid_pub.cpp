#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "pid_pubcpp");
    ros::NodeHandle nh;

    // Create a publisher for the "cmd_pid" topic with a queue size of 10
    ros::Publisher pid_pub = nh.advertise<geometry_msgs::Point>("cmd_pid", 10);

    // Set the loop rate to 10 Hz
    ros::Rate loop_rate(10);

    // Initialize PID values to zero
    double kp = 0.0, ki = 0.0, kd = 0.0;

    // Validate and retrieve PID values from command line arguments (if provided)
    if (argc >= 2)
    {
        // Convert the first command line argument to a double value
        kp = atof(argv[1]);
    }
    else
    {
        // If no command line arguments are provided, use default values
        ROS_INFO("No command line arguments provided for kp. Assume kp=0");
    }

    if (argc >= 3)
    {
        // Convert the second command line argument to a double value
        ki = atof(argv[2]);
    }
    else
    {
        // If no command line arguments are provided, use default values
        ROS_INFO("No command line arguments provided for ki. Assume ki=0");
    }

    if (argc >= 4)
    {
        // Convert the third command line argument to a double value
        kd = atof(argv[3]);
    }
    else
    {
        // If no command line arguments are provided, use default values
        ROS_INFO("No command line arguments provided for kd. Assume kd=0");
    }

    // Create a geometry_msgs::Point message to hold the PID values
    geometry_msgs::Point pid_msg;

    // Assign the PID values to the message
    pid_msg.x = kp;
    pid_msg.y = ki;
    pid_msg.z = kd;

    // wait for a subscriber to the 'cmd_pid' topic
    while (pid_pub.getNumSubscribers() == 0)
    {
        ROS_INFO("Waiting for a subscriber to the 'cmd_pid' topic...");
        ros::Duration(0.2).sleep();
    }

    // Log the PID values for debugging/monitoring
    ROS_INFO_STREAM("Publishing PID values: x = " << pid_msg.x << ", y = " << pid_msg.y << ", z = " << pid_msg.z);

    // Publish the PID message
    pid_pub.publish(pid_msg);

    // Process pending callbacks (optional)
    ros::spinOnce();

    // Sleep for the remaining time to maintain the desired loop rate
    loop_rate.sleep();

    // Keep publishing the message at the desired rate
    

    return 0;
}