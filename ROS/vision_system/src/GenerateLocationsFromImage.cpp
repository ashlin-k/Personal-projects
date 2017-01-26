#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <grid_map/grid_map.hpp>
#include <grid_map_msgs/GridMap.h>
#include <GridMapCvConverter.hpp>

using namespace cv;
using namespace std;

/*
Hue values of basic colors
Orange  0-22
Yellow 22- 38
Green 38-75
Blue 75-130
Violet 130-160
Red 160-179
*/

// gridmap is in text pg 103

Mat getImage(const char filename, string colourFilter)
{
  // filename = "/home/ashlin/Desktop/testbench_4.jpg"
  // get image from file
  Mat imgOriginal = imread(filename, CV_LOAD_IMAGE_COLOR);
  if (&imgOriginal == NULL)
  {
    ROS_INFO("Could not load image.");
    return -1;    
  }

  //namedWindow("Control", CV_WINDOW_NORMAL);
  //namedWindow("Thresholded_obstacles", CV_WINDOW_NORMAL);

  ROS_INFO("Image is not null.");

  int lastX = -1, lastY = -1;
  double posX = 0, posY = 0;

  Mat imgHSV, imgThresholded, imgLines;
  bool success;
  Moments mom;
  double mom01, mom10, momArea;

  imgLines = Mat::zeros(imgOriginal.size(), CV_8UC3);

  cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

  if (colourFilter[0] == 'b')   // black
  {
    inRange(imgHSV, Scalar(0, 0, 0), Scalar(180, 255, 30), imgThresholded);
  }
  else if (colourFilter[0] == 'w')    // white
  {
    inRange(imgHSV, Scalar(0, 0, 200), Scalar(180, 255, 255), imgThresholded);
  }
  else if (colourFilter[0] == 'g')    // green
  {
    inRange(imgHSV, Scalar(38, 100, 100), Scalar(75, 255, 255), imgThresholded);     
    
  } 
  else     // red
  {
    inRange(imgHSV, Scalar(160, 100, 100), Scalar(179, 255, 255), imgThresholded);     
    
  } 

  // morphological opening
  // ie. remove small objects from the foreground
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)));
  dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)));

  // morphological closing
  // ie. fill small holes in the foreground
  dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)));
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(5, 5))); 

  mom = moments(imgThresholded);
  mom01 = mom.m01;
  mom10 = mom.m10;
  momArea = mom.m00;

  if (momArea > 1000)
  {
    posX = mom10 / momArea;
    posY = mom01 / momArea;

    if (lastX >= 0 && lastY >= 0 && posX >= 0 && posX >= 0)
    {
      line(imgLines, Point(lastX, lastY), Point(posX, posY), Scalar(0, 0, 255), 2);
    }

    lastX = posX;
    lastY = posY;
  }

  imgOriginal = imgOriginal + imgLines;
  bitwise_not(imgThresholded, imgThresholded);

  // remove noise from thresholded image
  //fastNlMeansDenoising(imgThresholded, imgThresholded, 30.0, 7, 21);

  //imshow("Thresholded_obstacles", imgThresholded);
  //imshow("Control", imgOriginal);

  waitKey(0);

  return imgThresholded;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GenerateLocationsFromImage");

  ros::NodeHandle n;

  ros::Publisher msg_pub = n.advertise<std_msgs::String>("msg", 1000);
  ros::Publisher map_pub = n.advertise<grid_map_msgs::GridMap>("grid_map", 1000);

  std_msgs::String msgString;
  std::stringstream ss;
  ss << "hello world!";
  msgString.data = ss.str();
  ss.str(std::string());

  grid_map_msgs::GridMap msgGridmap;

  ros::Rate loop_rate(10);

  // get colour to filter from args
  string colourFilter = "";
  if (argc == 2)
  {
    colourFilter = argv[1];
  }

  Mat imgObstacles = getImage("/home/ashlin/Desktop/testbench_4.jpg", "green");  
  Mat imgRobots;

  // create base grid map
  // add cv layer of obstacles to base grid map
  GridMap envMap({"environtment"});
  map.setFrameId("map");
  GridMapCvConverter::addLayerFromImage(&imgObstacles, "environment", &envMap); 

  ROS_INFO("Finished image processing.");

  int count = 0;
  //while (ros::ok())
  {
    // get image of robots
    imgRobots = getImage("/home/ashlin/Desktop/testbench_robots.jpg", "red");

    // remove robot layer of map if it exists
    if (map.exists("robots"))
    {
      map.erase("robots");
    }

    // add cv layer of new robot positions to base grid map
    GridMapCvConverter::addLayerFromImage(&imgRobots, "robots", &envMap); 

    // convert GridMap to grid_map_msg
    GridMapRosConverter::toMessage(map, msgGridmap);

    msg_pub.publish(msgString);
    map_pub.publish(envMap);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}

