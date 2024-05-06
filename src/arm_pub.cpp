#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_pubcpp");
    ros::NodeHandle n;

    ros::Publisher arm_pub = n.advertise<std_msgs::Float32>("cmd_arm", 10);
    ros::Rate loop_rate(10);

    std_msgs::Float32 arm_msg;

    while (arm_pub.getNumSubscribers() == 0)
    {
        ROS_INFO("Waiting for a subscriber to the 'cmd_arm' topic...");
        ros::Duration(0.2).sleep();
    }

    if (argc > 1)
    {
        arm_msg.data = atof(argv[1]);
    }
    else
    {
        arm_msg.data = 0.0;
        ROS_INFO("No value provided so we assume value is zero ");

    }

    ROS_INFO_STREAM(arm_msg.data);

    arm_pub.publish(arm_msg);

    // Process pending callbacks (optional)
    ros::spinOnce();

    // Sleep for the remaining time to maintain the desired loop rate
    loop_rate.sleep();


    return 0;
}