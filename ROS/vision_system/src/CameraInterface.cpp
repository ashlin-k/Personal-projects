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

using namespace cv;
using namespace std;

Mat getFilteredImage(char* filename, string colourFilter);
nav_msgs::OccupancyGrid convertMatToOccGrid(Mat *m);
geometry_msgs::Pose getOrigin();
void calibrate(char *filename);
void findBorderCorners(Mat *src, Point2f *srcTri);
Mat setPerspective(Mat m);

Mat warp_mat;
unsigned int headerId = 0;
double resolution = 0;    // res in meters/pixel; set in calibrate()
#define TB_WIDTH_M 1.0      // test bench width in meters; y axis
#define TB_LENGTH_M 2.0      // test bench length in meters; x axis
unsigned int TB_WIDTH_PIX = 0;
unsigned int TB_LENGTH_PIX = 0;
unsigned int PIX_BUFFER = 5;

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

  // cant use images that are directly in the src folder for some reason
  // copy and paste them to your desktop
  char* fileEnv = "/home/ashlin/Desktop/opencv_images/border_corners_dark_blue.jpg";
  char* fileRobots = "/home/ashlin/Desktop/opencv_images/border_corners_dark_blue.jpg";
  char* calFile = "/home/ashlin/Desktop/opencv_images/border_corners_dark_blue.jpg";

  // calibrate camera for affine transform
  calibrate(calFile);

  Mat imgObstacles, imgRobots, imgWarped; 
  imgObstacles = getFilteredImage(fileEnv, "red");

  // create base occupancy grid and send it to subscriber
  nav_msgs::OccupancyGrid occGridEnv, occGridRobots;
  occGridEnv = convertMatToOccGrid(&imgObstacles);
  map_env_pub.publish(occGridEnv);
  ros::spinOnce();
  int i = 0;

  // while (ros::ok())
  // {
  //   // get image of robots
  //   imgRobots = getFilteredImage(fileRobots, "red");
  //   occGridRobots = convertMatToOccGrid(&imgRobots);

  //   msg_pub.publish(msgString);
  //   map_robots_pub.publish(occGridRobots);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   i++;
  // }

  ROS_INFO("Finished image processing.");

  return 0;
}

Mat getFilteredImage(char *filename, string colourFilter)
{
  // get image from file
  Mat imgOriginal = imread(filename, IMREAD_COLOR);
  if (&imgOriginal == NULL)
  {
    ROS_INFO("Could not load image.");
    Mat empty;
    return empty;    
  }

  // perspective transform
  Mat imgWarped;
  if (!warp_mat.empty())
  {
    imgWarped = setPerspective(imgOriginal);
  } 
  else
  {
    imgWarped = imgOriginal;
  } 

  ROS_INFO("Getting image from camera.");

  int lastX = -1, lastY = -1;
  double posX = 0, posY = 0;

  Mat imgHSV, imgThresholded, imgLines;
  bool success;
  Moments mom;
  double mom01, mom10, momArea;

  imgLines = Mat::zeros(imgWarped.size(), CV_8UC3);
  imgHSV = Mat::zeros(imgWarped.size(), CV_8UC3);

  // ROS_INFO("num channels: %d\n", imgOriginal.channels());

  cvtColor(imgWarped, imgHSV, COLOR_BGR2HSV);

  if (colourFilter == "black")   // black
  {
    inRange(imgHSV, Scalar(0, 0, 0), Scalar(180, 255, 30), imgThresholded);
  }
  else if (colourFilter == "white")    // white
  {
    inRange(imgHSV, Scalar(0, 0, 200), Scalar(180, 255, 255), imgThresholded);
  }
  else if (colourFilter == "green")    // green
  {
    inRange(imgHSV, Scalar(38, 100, 100), Scalar(75, 255, 255), imgThresholded);     
    
  }
  else if (colourFilter == "blue")    // blue
  {
    inRange(imgHSV, Scalar(75, 100, 100), Scalar(130, 255, 255), imgThresholded);     
    
  }  
  else if (colourFilter == "red")     // red
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

  imgWarped = imgWarped + imgLines;
  bitwise_not(imgThresholded, imgThresholded);

  // show images
  // namedWindow("Control", CV_WINDOW_NORMAL);
  // namedWindow("Thresholded", CV_WINDOW_NORMAL);
  // imshow("Control", imgWarped);
  // imshow("Thresholded", imgThresholded);
  // waitKey(0);

  return imgThresholded;
}

Mat setPerspective(Mat m)
{
  if ((TB_LENGTH_PIX != 0) & (TB_WIDTH_PIX != 0) & (!warp_mat.empty()))
  {
    Mat warp_dst = Mat::zeros( TB_LENGTH_PIX + 2*PIX_BUFFER, TB_WIDTH_PIX + 2*PIX_BUFFER, m.type() );
    warpPerspective( m, warp_dst, warp_mat, warp_dst.size() );

    return warp_dst;
  }
  
  return m;
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
  occ.header.frame_id = "map";

  // create MapMetaData
  occ.info.map_load_time = current_time;
  occ.info.resolution = resolution;      // meters / pixel
  occ.info.width = m->cols;
  occ.info.height = m->rows;
  occ.info.origin = getOrigin();

  // create int8[]
  Mat img8S = Mat::zeros(m->size(), CV_8SC1);
  m->convertTo(img8S, CV_8SC1);
  vector<signed char> imgArray;
  if (img8S.isContinuous()) 
  {
    imgArray.assign(img8S.datastart, img8S.dataend);
  } 
  else 
  {
    for (int i = 0; i < img8S.rows; ++i) 
    {
      imgArray.insert(imgArray.end(), img8S.ptr<signed char>(i), img8S.ptr<signed char>(i)+img8S.cols);
    }
  }

  occ.data = imgArray;

  return occ;
}


geometry_msgs::Pose getOrigin()
{
  geometry_msgs::Pose p;
  p.position.x = 0;
  p.position.y = 0;
  p.position.z = 0;
  p.orientation = tf::createQuaternionMsgFromYaw(0);

  return p;
}

void findBorderCorners(Mat *src, Point2f *srcTri)
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  int thresh = 100;
  int max_thresh = 255;

  /// Detect edges using canny
  Canny( *src, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
    { 
      mu[i] = moments( contours[i], false ); 
    }

  ///  Get the mass centers
  if (contours.size() == 4)
  {
    printf("Detected corners successfully!\n");
    for( int i = 0; i < 4; i++ )
    {
      srcTri[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }
  }
  else
  {   
    // Noticed that for the circles, it detected each circle twice in a row, so there will be 6 contours
    // Therefore, take every other circle from contours

    printf("Calibration error: Detected %lu corners.\n", contours.size());

    // vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size()/2; i++ )
    { 
      // mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
      srcTri[i] = Point2f( mu[2*i].m10/mu[2*i].m00 , mu[2*i].m01/mu[2*i].m00 ); 
    }

    /// Draw contours - for debugging
    // Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    // Scalar color[] = { Scalar(0,0,255), Scalar(0,255,0), Scalar(255,0,0), 
    //   Scalar(255,0,255), Scalar(0,255,255), Scalar(255,255,0)};
    //   namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    // for( int i = 0; i< contours.size(); i++ )
    // {
    //   // Scalar color = Scalar( 0,0,255 );
    //   drawContours( drawing, contours, i, color[i%6], 2, 8, hierarchy, 0, Point() );
    //   imshow( "Contours", drawing );
    //   waitKey(0);
    //   // circle( drawing, mc[i], 20, color[i], 5, 8, 0 );
    // }

  }

  printf ("Cal points:\npoint 1: %.2f, %.2f\npoint 2: %.2f, %.2f\npoint 3: %.2f, %.2f\npoint 4: %.2f, %.2f\n", 
    srcTri[0].x, srcTri[0].y, srcTri[1].x, srcTri[1].y, srcTri[2].x, srcTri[2].y, srcTri[3].x, srcTri[3].y);
}

void calibrate(char *filename)
{
  Point2f srcTri[4];
  Point2f dstTri[4];    

  Mat original, src, warp_dst;
  // Mat warp_rotate_dst;

  /// Load the image
  // get image filtered for border corners colour
  original = imread(filename, IMREAD_COLOR);
  Mat orgGray = imread(filename, IMREAD_GRAYSCALE);
  src = getFilteredImage(filename, "blue");

  /// Set your 3 points to calculate the  Affine Transform
  // assume width is the base of the table closest to the camera. 
  // we will use the ratio of width:height to calculate height  
  findBorderCorners(&src, srcTri);
  TB_WIDTH_PIX = abs(srcTri[1].x - srcTri[0].x);
  TB_LENGTH_PIX = TB_WIDTH_PIX * (TB_LENGTH_M/TB_WIDTH_M);
  PIX_BUFFER = 5;

  dstTri[0] = Point2f( PIX_BUFFER, PIX_BUFFER + TB_LENGTH_PIX ); // (0,max)
  dstTri[1] = Point2f( PIX_BUFFER + TB_WIDTH_PIX, PIX_BUFFER + TB_LENGTH_PIX );  //(max, max)
  dstTri[2] = Point2f( PIX_BUFFER + TB_WIDTH_PIX, PIX_BUFFER ); //(max, 0)
  dstTri[3] = Point2f( PIX_BUFFER, PIX_BUFFER ); //(0, 0)


  // (0,0)
  // ------------> X, width
  // |
  // |
  // |
  // |
  // |
  // v
  // Y, height

  // (0, 0)                 (max, 0)
  // pt 3                   pt 2
  // -------------------------
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // |                       |
  // -------------------------
  // pt 0                   pt 1
  // (0, max)               (max, max)

  /// Set the dst image the same type and size as src
  warp_dst = Mat::zeros( TB_LENGTH_PIX + 2*PIX_BUFFER, TB_WIDTH_PIX + 2*PIX_BUFFER, src.type() );


  /// Get the Perspective Transform
  warp_mat = getPerspectiveTransform( srcTri, dstTri );

  /// Apply the Perspective Transform just found to the src image
  warpPerspective( orgGray, warp_dst, warp_mat, warp_dst.size() );

  /** Rotating the image after Warp */
  /// Compute a rotation matrix with respect to the center of the image
  // Point center = Point( warp_dst.cols/2, warp_dst.rows/2 );
  // double angle = 0.0;
  // double scale = 1.0;
  /// Get the rotation matrix with the specifications above
  // rot_mat = getRotationMatrix2D( center, angle, scale );
  /// Rotate the warped image; don't need this right now, but maybe will later
  // warpAffine( warp_dst, warp_rotate_dst, *\rot_mat, warp_dst.size() );

  // show images
  // namedWindow("Original", CV_WINDOW_NORMAL);
  // namedWindow("Filtered", CV_WINDOW_NORMAL);
  // namedWindow("Affine", CV_WINDOW_NORMAL);
  // imshow("Original", orgGray);
  // imshow("Filtered", src);
  // imshow("Affine", warp_dst);
  // waitKey(0);

  // set the resolution
  double resWidth = TB_WIDTH_M / warp_dst.cols;
  double resHeight = TB_LENGTH_M / warp_dst.rows;
  // printf("res_width = %.6f, res_height = %.6f\n", resWidth, resHeight);
  resolution = (resWidth + resHeight) / 2;

}