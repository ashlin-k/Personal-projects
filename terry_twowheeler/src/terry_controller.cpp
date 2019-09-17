#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <fstream>
#include <string.h>

ros::Publisher rightWheelPub, leftWheelPub;
std_msgs::Float64 rightWheelEffort, leftWheelEffort;

std::ofstream myfile;
std::string dataFile = "/home/ashlin/catkin_ws/src/terry_twowheeler/data.csv";

void SetWheelTorques(const sensor_msgs::Imu::ConstPtr &imu)
{
    std_msgs::Float64 rightWheelCmd, leftWheelCmd;
    static double trq = 0;

    if (trq < 10)
    {
        rightWheelCmd.data = trq;
        leftWheelCmd.data = trq;

        ros::Time now = ros::Time::now();

        rightWheelPub.publish(rightWheelCmd);
        leftWheelPub.publish(leftWheelCmd);

        myfile.open (dataFile, std::ofstream::out | std::ofstream::app);
        // write time, trqocity, acc_x, acc_y, acc_z
        myfile << now.toSec() << "," << trq << "," <<
            imu->linear_acceleration.x << "," <<
            imu->linear_acceleration.y << "," 
            << imu->linear_acceleration.z << "\n";
        myfile.close();

        trq+=0.05;
    }

    else
    {
        rightWheelCmd.data = 0;
        leftWheelCmd.data = 0;

        rightWheelPub.publish(rightWheelCmd);
        leftWheelPub.publish(leftWheelCmd);
    }

    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "terry_controller");
    ros::NodeHandle rosnode;

    // Waits for simulation time update.
    ros::Time last_ros_time_;
    bool wait = true;
    while (wait)
    {
        last_ros_time_ = ros::Time::now();
        if (last_ros_time_.toSec() > 0)
        wait = false;
    }

    // ros topic subscriptions
    ros::SubscribeOptions imuSubOpt =
        ros::SubscribeOptions::create<sensor_msgs::Imu>(
        "/terry/imu", 1, SetWheelTorques,
        ros::VoidPtr(), rosnode.getCallbackQueue());

    // Because TCP causes bursty communication with high jitter,
    // declare a preference on UDP connections for receiving
    // joint states, which we want to get at a high rate.
    // Note that we'll still accept TCP connections for this topic
    // (e.g., from rospy nodes, which don't support UDP);
    // we just prefer UDP.
    imuSubOpt.transport_hints = ros::TransportHints().unreliable();

    ros::Subscriber subImu = rosnode.subscribe(imuSubOpt);

    rightWheelPub = rosnode.advertise<std_msgs::Float64>
        ("/terry/right_wheel_controller/command", 1000);
    leftWheelPub = rosnode.advertise<std_msgs::Float64>
        ("/terry/left_wheel_controller/command", 1000);

    // create a new file to record imu data
    myfile.open (dataFile, std::ofstream::out | std::ofstream::trunc);
    myfile.close();

    ros::spin();

    return 0;
}