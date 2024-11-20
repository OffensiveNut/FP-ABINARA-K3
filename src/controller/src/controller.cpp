#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

bool scan()
{
    bool found = false;
    // while(!found){
    //     //
    // }
}

void moveto(int x, int y, int z)
{
}

int main(int argc, char **argv)
{
// Initialize ROS node
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

// Create a publisher object
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("servo", 10);
    
    // Set the loop rate (10 Hz)
    ros::Rate loop_rate(100);

    int nomor = 0;
    while (ros::ok())
    {
        // Create and populate the message
            std_msgs::Int32MultiArray msg;
            msg.data = {0,0,0,0,0}; // Sample array of integers

        // Publish the message
            pub.publish(msg);
            
        ROS_INFO("Published array: [0,0,0,0,0]");

        // Call any callbacks and sleep for the remaining time
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
