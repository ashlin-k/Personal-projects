#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
// #include <grid_map/grid_map.hpp>
// #include <grid_map_msgs/GridMap.h>
// #include <GridMapCvConverter.hpp>

using namespace cv;
using namespace std;

Mat getImage(char* filename, string colourFilter);
nav_msgs::OccupancyGrid convertMatToOccGrid(Mat *m);
double getResolution(Mat *m);
geometry_msgs::Pose getOrigin(Mat *m);

unsigned int headerId = 0;
#define TB_WIDTH_M 1.0      // test bench width in meters; y axis
#define TB_LENGTH_M 2.0      // test bench length in meters; x axis

/*
Hue values of basic colors
Orange  0-22
Yellow 22- 38
Green 38-75
Blue 75-130
Violet 130-160
Red 160-179
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GenerateLocationsFromImage");

  ros::NodeHandle n;

  ros::Publisher msg_pub = n.advertise<std_msgs::String>("msg", 1000);
  ros::Publisher map_env_pub = n.advertise<nav_msgs::OccupancyGrid>("occ_grid_env", 1000);
  ros::Publisher map_robots_pub = n.advertise<nav_msgs::OccupancyGrid>("occ_grid_robots", 1000);

  std_msgs::String msgString;
  std::stringstream ss;
  ss << "hello world!";
  msgString.data = ss.str();
  ss.str(std::string());

  ros::Rate loop_rate(10);

  // get colour to filter from args
  string colourFilter = "";
  if (argc == 2)
  {
    colourFilter = argv[1];
  }

  char* fileEnv = "testbench_4.jpg";
  char* fileRobots = "testbench_robots.jpg";

  Mat imgObstacles, imgRobots;
  imgObstacles = getImage(fileEnv, "green");  

  // create base occupancy grid and send it to subscriber
  nav_msgs::OccupancyGrid occGridEnv, occGridRobots;
  occGridEnv = convertMatToOccGrid(&imgObstacles);
  map_env_pub.publish(occGridEnv);
  ros::spinOnce();


  int count = 0;
  //while (ros::ok())
  {
    // get image of robots
    imgRobots = getImage(fileRobots, "red");
    occGridRobots = convertMatToOccGrid(&imgRobots);

    msg_pub.publish(msgString);
    map_robots_pub.publish(occGridRobots);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  ROS_INFO("Finished image processing.");

  return 0;
}

Mat getImage(char *filename, string colourFilter)
{
  // char *f = "/home/ashlin/Desktop/testbench_4.jpg";
  // get image from file
  Mat imgOriginal = imread(filename, CV_LOAD_IMAGE_COLOR);
  if (&imgOriginal == NULL)
  {
    ROS_INFO("Could not load image.");
    Mat empty;
    return empty;    
  }

  // create windows for images
  namedWindow("Control", CV_WINDOW_NORMAL);
  namedWindow("Thresholded", CV_WINDOW_NORMAL);

  ROS_INFO("Image is not null.");

  int lastX = -1, lastY = -1;
  double posX = 0, posY = 0;

  Mat imgHSV, imgThresholded, imgLines;
  bool success;
  Moments mom;
  double mom01, mom10, momArea;

  imgLines = Mat::zeros(imgOriginal.size(), CV_8UC3);
  imgHSV = Mat::zeros(imgOriginal.size(), CV_8UC3);

  // ROS_INFO("num channels: %d\n", imgOriginal.channels());

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
    // this 0-10 range detects lower red hues; the 160-179 range detects upper red hues
    inRange(imgHSV, Scalar(0, 100, 100), Scalar(10, 255, 255), imgThresholded);    
    
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

  imshow("Control", imgOriginal);
  imshow("Thresholded", imgThresholded);

  waitKey(0);

  return imgThresholded;
}

nav_msgs::OccupancyGrid convertMatToOccGrid(Mat *m)
{
  nav_msgs::OccupancyGrid occ;

  // get current time
  ros::Time current_time = ros::Time::now();

  // create Header
  occ.header.seq = headerId;
  headerId++;
  occ.header.stamp = current_time;
  occ.header.frame_id = "occ_grid";

  // create MapMetaData
  occ.info.map_load_time = current_time;
  occ.info.resolution = getResolution(m);      // meters / pixel
  occ.info.width = m->cols;
  occ.info.height = m->rows;
  occ.info.origin = getOrigin(m);

  // create int8[]
  Mat img8S;
  img8S = Mat::zeros(m->size(), CV_8S);
  m->convertTo(img8S, CV_8S);

  return occ;
}

// still in progress
double getResolution(Mat *m)
{
  // get inner rectangle from border 
  // get length of rectangle in pixels, L_pix
  // res(m/pix) = TB_LENGTH_M(m) / L_pix(pix)

  return 0.00001;
}


// still in progress
geometry_msgs::Pose getOrigin(Mat *m)
{
  double x = 0;
  double y = 0;
  double z = 0;
  double theta = 0;

  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);

  return p;
}