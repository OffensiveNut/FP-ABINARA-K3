#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <mutex>
#include <cmath>

using namespace std;

// Member function to publish servo angles
void publishServoAngles(ros::Publisher &pub, const vector<double> &ik_result, double current_dist)
{
    std_msgs::Float64MultiArray msg;
    msg.data = ik_result;
    pub.publish(msg);
    ROS_INFO("Published angles: [servolink1: %f, servolink2: %f, servolink3: %f], Distance: %f mm\n",
             msg.data[0], msg.data[1], msg.data[2], current_dist);
}

// Member function to publish joint states
void publishJointStates(ros::Publisher &joint_pub,const double base, const vector<double> &ik_result,const double gripper, double current_dist)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"servopiringan", "servolink1", "servolink2", "servolink3", "servogear"};
    joint_state.position = {gripper, ik_result[0], ik_result[1], ik_result[2], base}; // Adjust as needed

    joint_pub.publish(joint_state);
    ROS_INFO("Published joint states: [servopiringan: %lf rad, servolink1: %lf rad, servolink2: %lf rad, servolink3: %lf rad, servogear: %lf rad], Distance: %lf mm\n",
             joint_state.position[0], joint_state.position[1], joint_state.position[2],
             joint_state.position[3], joint_state.position[4], current_dist);
}

class RobotArm
{
private:
    std::mutex dist_mutex;
    double current_distance;

public:
    // in mm
    const double L1 = 116.0; 
    const double L2 = 94.65;
    const double L3 = 117.5;

    RobotArm() : current_distance(0.0) {}

    void distCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        lock_guard<mutex> lock(dist_mutex);
        current_distance = msg->data;
        // ROS_INFO("Received distance: %f", current_distance);
    }

    double getDistance()
    {
        lock_guard<mutex> lock(dist_mutex);
        return current_distance;
    }

    double forwardKinematicsX(double theta1, double theta2, double theta3)
    {
        return L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3);
    }

    double forwardKinematicsY(double theta1, double theta2, double theta3)
    {
        return L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3);
    }

    vector<double> inverseKinematics(double target_x, double target_y)
    {
        double x = target_x;
        double y = target_y;

        double r = sqrt(x * x + y * y);
        if (r > (L1 + L2) || r < fabs(L1 - L2))
        {
            ROS_WARN("Position (%.2f, %.2f) is unreachable", target_x, target_y);
            return {0.0, 0.0, 0.0};
        }

        double cos_theta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        if (cos_theta2 > 1.0 || cos_theta2 < -1.0)
        {
            ROS_WARN("No solution exists for this position");
            return {0.0, 0.0, 0.0};
        }

        double theta2 = acos(cos_theta2);
        double k1 = L1 + L2 * cos(theta2);
        double k2 = L2 * sin(theta2);
        double theta1 = atan2(y, x) - atan2(k2, k1);
        double theta3 = -(theta1 + theta2);

        return {theta1, theta2, theta3};
    }
    void scan(ros::Publisher &joint_pub)
    {
        bool isFound = false;
        ros::Rate rate(10); // 10Hz for smooth motion

        // Constants
        constexpr double rad90 = M_PI / 2.0;
        constexpr double THRESHOLD_DIST = 200.0; // 200mm (20cm)

        // Initial scanning position (-90°, 90°, -90°)
        vector<double> initial_angles = {-rad90, -rad90, rad90};
        publishJointStates(joint_pub,rad90, initial_angles,rad90/2, getDistance());

        // Scanning angle for servopiringan (first joint)
        double scan_angle = -rad90;   // Start from -90°
        constexpr double STEP = 0.05; // Small increment for smooth motion
        bool direction = true;        // true = right, false = left

        while (!isFound && ros::ok())
        {
            // Update servopiringan angle (first joint)
            scan_angle += direction ? STEP : -STEP;

            // Reverse direction if limits are reached
            if (scan_angle >= rad90)
            {
                scan_angle = rad90;
                direction = false;
            }
            else if (scan_angle <= -rad90)
            {
                scan_angle = -rad90;
                direction = true;
            }

            // Publish joint states
            publishJointStates(joint_pub,scan_angle, initial_angles,-rad90, getDistance());

            // Check distance
            double current_dist = getDistance();
            if (current_dist < THRESHOLD_DIST)
            {
                isFound = true;
                ROS_INFO("Object found at angle: %f degrees", scan_angle * 180.0 / M_PI);
                break;
            }

            rate.sleep();
            ros::spinOnce();
        }

        if (isFound)
        {
            // Optionally, perform additional actions when object is found
            ROS_INFO("Scanning completed. Object detected.");
        }
        else
        {
            ROS_WARN("Scanning terminated without finding the object.");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // Use 2 threads

    RobotArm robotArm;

    // Publisher for servo angles
    // ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("servo", 10);
    // Publisher for joint states
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    // Subscriber for distance
    ros::Subscriber dist_sub = nh.subscribe("dist", 10,
                                            &RobotArm::distCallback, &robotArm);

    spinner.start();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //===========================debug================
        // Vector to store joint angles in degrees
        double base_deg, capit_deg;
        double joint1_deg, joint2_deg, joint3_deg;
        double current_dist;
        
        // Prompt user for input
        std::cout << "Enter capit, wrist, elbow, shoulder, base angles: ";
        std::cin >> capit_deg >> joint3_deg >> joint2_deg >> joint1_deg >> base_deg;
        
        // Convert degrees to radians
        double base_rad = base_deg * (M_PI / 180.0);
        double joint1_rad = joint1_deg * (M_PI / 180.0);
        double joint2_rad = joint2_deg * (M_PI / 180.0);
        double joint3_rad = joint3_deg * (M_PI / 180.0);
        double capit_rad = capit_deg * (M_PI / 180.0);
        
        // Store converted angles in vector
        std::vector<double> ik_result = {joint1_rad, joint2_rad, joint3_rad};
        
        // Access distance data when needed
        current_dist = robotArm.getDistance(); /* your method to get distance, e.g., robotArm.getDistance() */;
        
        // Publish joint states with converted radian values
        publishJointStates(joint_pub, capit_rad, ik_result, base_rad, current_dist);
        
        loop_rate.sleep();
        //===========================debug================

        // std_msgs::Float64MultiArray msg;
        // // vector<double> ik_result = robotArm.inverseKinematics(15, 10);
        // vector<double> ik_result = {0.0,0.0,0.0};
        // msg.data = ik_result;

        // // Access distance data when needed
        // double current_dist = robotArm.getDistance();
        // double base,capit;
        // cin >> base >> ik_result[0] >> ik_result[1] >> ik_result[2] >> capit;
        // // Publish servo angles
        // // publishServoAngles(pub, ik_result, current_dist);

        // // Publish joint states
        // publishJointStates(joint_pub,base, ik_result,capit, current_dist);

        // robotArm.scan(joint_pub);

        // loop_rate.sleep();
    }
    return 0;
};
