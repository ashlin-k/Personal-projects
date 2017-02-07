#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

void messageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I received: [%s]", msg->data.c_str());
}

void envOccGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  ROS_INFO("I received: env occ grid.");
}

void robotsOccGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  ROS_INFO("I received: robot occ grid.");
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber msg_sub = n.subscribe("msg", 1000, messageCallback);
  ros::Subscriber map_env_sub = n.subscribe("occ_grid_env", 1000, envOccGridCallback);
  ros::Subscriber map_robots_sub = n.subscribe("occ_grid_robots", 1000, robotsOccGridCallback);

  ros::spin();

  return 0;
}