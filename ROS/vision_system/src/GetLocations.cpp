#include "ros/ros.h"
#include "std_msgs/String.h"
#include <grid_map/grid_map.hpp>
#include <grid_map_msgs/GridMap.h>

void messageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void gridmapCallback(const grid_map_msgs::GridMap::ConstPtr& map)
{
  ROS_INFO("I heard: grid map.");
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber msg_sub = n.subscribe("msg", 1000, messageCallback);
  ros::Subscriber msg_sub = n.subscribe("grid_map", 1000, gridmapCallback);

  ros::spin();

  return 0;
}