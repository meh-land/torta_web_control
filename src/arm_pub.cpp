#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "Arm_Positions");

    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("Arm_Positions_Publisher", 10);


    // Set the loop rate (in Hz)
    ros::Rate loop_rate(1); // 1 Hz

    // Counter variable
    int count = 0;
    int grip=0;
    while (ros::ok()) {
        // Create a message object
        std_msgs::Int32MultiArray msg;

        // Set the array size to 3
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = 3;
        msg.layout.dim[0].stride = 3;
        msg.layout.dim[0].label = "Arm_Positions";

        // Add the integer values to the data array
        msg.data.clear();
        msg.data.push_back(0);
        msg.data.push_back(0);
        msg.data.push_back(grip);
	
        // Publish the message
        pub.publish(msg);

        // Log the published values
        ROS_INFO("Published: [%d, %d, %d]", msg.data[0], msg.data[1], msg.data[2]);

        // Increment the counter
        count += 45; // Increment by 3 for the next set of values
        if (grip == 0)
        {
        	grip = 1;
        }
        else
        {
        	grip = 0;
        }
        
        // Spin once
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
