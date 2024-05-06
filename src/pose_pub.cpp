#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_pubcpp");
    ros::NodeHandle nh;

    // Create a publisher for the cmd_pose topic
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("cmd_pose", 10);

    // Get the Pose values from command-line arguments
    geometry_msgs::Pose pose_msg;
    

    if (argc > 1) {
        pose_msg.position.x = atof(argv[1]);
    } else {
        pose_msg.position.x = 0.0;
        ROS_INFO("No value provided for x, using default value: 0.0");
    }

    if (argc > 2) {
        pose_msg.position.y = atof(argv[2]);
    } else {
        pose_msg.position.y = 0.0;
        ROS_INFO("No value provided for y, using default value: 0.0");
    }

    if (argc > 3) {
        pose_msg.orientation.w = atof(argv[3]);
    } else {
        pose_msg.orientation.w = 0.0;
        ROS_INFO("No value provided for theta, using default value: 0.0");
    }

    // wait for a subscriber to the 'cmd_pose' topic
    ROS_INFO("Waiting for a subscriber to the 'cmd_pose' topic...");
    while (pose_pub.getNumSubscribers() == 0) {
        ros::Duration(0.2).sleep();
    }

    // Print the Pose values
    ROS_INFO_STREAM("Publishing Pose: x=" << pose_msg.position.x << ", y=" << pose_msg.position.y << ", theta=" << pose_msg.orientation.w);

    // Publish the Pose message
    pose_pub.publish(pose_msg);

    // Use a ros::Rate object to control the publishing rate
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    return 0;
}