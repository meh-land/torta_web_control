#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main (int argc,char **argv)
{
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub=n.advertise<geometry_msgs::Twist>("chatter",1000);
    ros::Rate loop_rate(1);


    while(ros::ok())
    {
        geometry_msgs::Twist Speed;
        Speed.linear.x=4.0;
        Speed.angular.z=2.0;
        ROS_INFO("Linear Velocity: x = %f, y = %f, z = %f", Speed.linear.x, Speed.linear.y, Speed.linear.z);
        ROS_INFO("Angular Velocity: x = %f, y = %f, z = %f", Speed.angular.x, Speed.angular.y, Speed.angular.z);

        chatter_pub.publish(Speed);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}