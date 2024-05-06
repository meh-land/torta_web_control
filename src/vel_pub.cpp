#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

/**
 * Main function of the vel_pub node
 */
int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "vel_pub");

    // Create a ROS node handle
    ros::NodeHandle n;

    // Initialize velocity values to 0.0
    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_theta = 0.0;

    // Check if command-line arguments are provided
    if (argc > 1)
    {
        if (argv[1][0] == '\0')
        { // check if the argument is empty
            ROS_INFO("No value provided for linear x velocity, assuming 0.0");
            vel_x = 0.0;
        }
        else
        {
            try
            {
                vel_x = std::stod(argv[1]);
            }
            catch (const std::invalid_argument &e)
            {
                ROS_INFO("Invalid argument for linear x velocity, assuming 0.0");
                vel_x = 0.0;
            }
        }
    }
    else
    {
        ROS_INFO("No value provided for linear x velocity, assuming 0.0");
        vel_x = 0.0;
    }

    if (argc > 2)
    {
        if (argv[2][0] == '\0')
        { // check if the argument is empty
            ROS_INFO("No value provided for linear y velocity, assuming 0.0");
            vel_y = 0.0;
        }
        else
        {
            try
            {
                vel_y = std::stod(argv[2]);
            }
            catch (const std::invalid_argument &e)
            {
                ROS_INFO("Invalid argument for linear y velocity, assuming 0.0");
                vel_y = 0.0;
            }
        }
    }
    else
    {
        ROS_INFO("No value provided for linear y velocity, assuming 0.0");
        vel_y = 0.0;
    }

    if (argc > 3)
    {
        if (argv[3][0] == '\0')
        { // check if the argument is empty
            ROS_INFO("No value provided for angular z velocity, assuming 0.0");
            vel_theta = 0.0;
        }
        else
        {
            try
            {
                vel_theta = std::stod(argv[3]);
            }
            catch (const std::invalid_argument &e)
            {
                ROS_INFO("Invalid argument for angular z velocity, assuming 0.0");
                vel_theta = 0.0;
            }
        }
    }
    else
    {
        ROS_INFO("No value provided for angular z velocity, assuming 0.0");
        vel_theta = 0.0;
    }

    // Create a publisher for the 'cmd_vel' topic
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // Set the publishing rate to 10 Hz
    ros::Rate loop_rate(10);

    // Create a geometry_msgs::Twist message
    geometry_msgs::Twist Speed;
    Speed.linear.x = vel_x;
    Speed.linear.y = vel_y;
    Speed.angular.z = vel_theta;

    // Print information about the velocity values
    ROS_INFO("Linear Velocity: x = %f, y = %f, z = %f", Speed.linear.x, Speed.linear.y, Speed.linear.z);
    ROS_INFO("Angular Velocity: x = %f, y = %f, z = %f", Speed.angular.x, Speed.angular.y, Speed.angular.z);

    // Wait for a subscriber to the 'cmd_vel' topic
    while (chatter_pub.getNumSubscribers() == 0)
    {
        ROS_INFO("Waiting for a subscriber to the 'cmd_vel' topic...");
        ros::Duration(0.2).sleep();
    }

    // Publish the velocity message
    chatter_pub.publish(Speed);

    // Spin once to process any callbacks
    ros::spinOnce();

    // Sleep for the remainder of the publishing period
    loop_rate.sleep();

    return 0;
}