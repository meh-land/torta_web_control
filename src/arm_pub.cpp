#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_pubcpp");
    ros::NodeHandle n;

    ros::Publisher arm_pub = n.advertise<std_msgs::Int32MultiArray>("Arm_Positions_Publisher", 10);
    ros::Rate loop_rate(10);

    std_msgs::Int32MultiArray arm_msg;

    arm_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    arm_msg.layout.dim[0].size = 3;
    arm_msg.layout.dim[0].stride = 3;
    arm_msg.layout.dim[0].label = "Arm_Positions";

    while (arm_pub.getNumSubscribers() == 0)
    {
        ROS_INFO("Waiting for a subscriber to the 'cmd_arm' topic...");
        ros::Duration(0.2).sleep();
    }

    if (argc > 3) // Check for at least three command-line arguments
    {
        arm_msg.data.push_back(atoi(argv[1]));
        arm_msg.data.push_back(atoi(argv[2]));
        arm_msg.data.push_back(atoi(argv[3]));
    }
    else
    {
        ROS_INFO("Insufficient values provided. Publishing default values.");
        arm_msg.data = {0, 0, 0};
    }

    ROS_INFO_STREAM("Publishing arm positions: " << arm_msg.data[0] << ", " << arm_msg.data[1] << ", " << arm_msg.data[2]);

    arm_pub.publish(arm_msg);

    // Process pending callbacks (optional)
    ros::spinOnce();

    // Sleep for the remaining time to maintain the desired loop rate
    loop_rate.sleep();

    return 0;
}
