#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <fstream>
#include <string.h>

ros::Publisher twistPub;

std::ofstream myfile;
std::string dataFile = "/home/ashlin/catkin_ws/src/terry_twowheeler/data_vel.csv";

void SetWheelVelocities(const sensor_msgs::Imu::ConstPtr &imu)
{
    geometry_msgs::Twist twistCmd;
    static double vel = 0;

    if (vel < 10)
    {
        twistCmd.linear.x = vel;

        ros::Time now = ros::Time::now();

        twistPub.publish(twistCmd);

        myfile.open (dataFile, std::ofstream::out | std::ofstream::app);
        // write time, velocity, acc_x, acc_y, acc_z
        myfile << now.toSec() << "," << vel << "," <<
            imu->linear_acceleration.x << "," <<
            imu->linear_acceleration.y << "," 
            << imu->linear_acceleration.z << "\n";
        myfile.close();

        vel+=0.05;
    }

    else
    {
        twistCmd.linear.x = 0;
        twistPub.publish(twistCmd);
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
        "/terry/imu", 1, SetWheelVelocities,
        ros::VoidPtr(), rosnode.getCallbackQueue());

    // Because TCP causes bursty communication with high jitter,
    // declare a preference on UDP connections for receiving
    // joint states, which we want to get at a high rate.
    // Note that we'll still accept TCP connections for this topic
    // (e.g., from rospy nodes, which don't support UDP);
    // we just prefer UDP.
    imuSubOpt.transport_hints = ros::TransportHints().unreliable();

    ros::Subscriber subImu = rosnode.subscribe(imuSubOpt);

    twistPub = rosnode.advertise<geometry_msgs::Twist>
        ("/terry/mobile_base_controller/cmd_vel", 1000);

    // create a new file to record imu data
    myfile.open (dataFile, std::ofstream::out | std::ofstream::trunc);
    myfile.close();

    ros::spin();

    return 0;
}