#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GenerateLocationsFromImage");

  ros::NodeHandle n;

  ros::Publisher corners_pub = n.advertise<std_msgs::String>("corner_locations", 1000);
  ros::Publisher landmarks_pub = n.advertise<std_msgs::String>("landmark_locations", 1000);
  ros::Publisher robots_pub = n.advertise<std_msgs::String>("robot_locations", 1000);

  ros::Rate loop_rate(10);

  // get image from file
  IplImage *img = cvLoadImage("/home/ashlin/Desktop/testbench_1.png");
  if (img == NULL)
  {
    return -1;    
  }

  ROS_INFO("Image is not null.");

  // show raw image
  cvNamedWindow("Raw");
  cvShowImage("Raw", img);

  //convert original image to greyscale
  IplImage *imgGreyscale = cvCreateImage(cvGetSize(img), 8, 1);
  cvCvtColor(img, imgGreyscale, CV_BGR2GRAY);

  // threshold the greyscal image to get better results
  cvThreshold(imgGreyscale, imgGreyscale, 128, 255, CV_THRESH_BINARY);

  // create pointers to contour and points, and store them in a memory block
  CvSeq *contours, *result;
  CvMemStorage *storage = cvCreateMemStorage(0);

  // find all contours in the image
  cvFindContours(imgGreyscale, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

  unsigned int numTriangles = 0;
  unsigned int numQuads = 0;
  unsigned int numPentagons = 0;

  while (contours)
  {   

    // obtain a sequence of points of contours, pointed to by the variable contours
    result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 
      cvContourPerimeter(contours)*0.02, 0);
    
    // if triangle
    if (result->total == 3)
    {
      numTriangles++;
      CvPoint *pt[3];
      for (int i = 0; i < 3; i++)
      {
        pt[i] = (CvPoint*)cvGetSeqElem(result, i);
      }

      // draw lines
      cvLine(img, *pt[0], *pt[1], cvScalar(0, 0, 255), 4);
      cvLine(img, *pt[1], *pt[2], cvScalar(0, 0, 255), 4);
      cvLine(img, *pt[2], *pt[0], cvScalar(0, 0, 255), 4);
    }

    // if quadrilateral
    else if (result->total == 4)
    {
      numQuads++;
      CvPoint *pt[4];
      for (int i = 0; i < 4; i++)
      {
        pt[i] = (CvPoint*)cvGetSeqElem(result, i);
      }

      // draw lines
      cvLine(img, *pt[0], *pt[1], cvScalar(255, 0, 0), 4);
      cvLine(img, *pt[1], *pt[2], cvScalar(255, 0, 0), 4);
      cvLine(img, *pt[2], *pt[3], cvScalar(255, 0, 0), 4);
      cvLine(img, *pt[3], *pt[0], cvScalar(255, 0, 0), 4);
    }

    // if pentagon
    else if (result->total == 5)
    {
      numPentagons++;
      CvPoint *pt[5];
      for (int i = 0; i < 5; i++)
      {
        pt[i] = (CvPoint*)cvGetSeqElem(result, i);
      }

      // draw lines
      cvLine(img, *pt[0], *pt[1], cvScalar(0, 255, 0), 4);
      cvLine(img, *pt[1], *pt[2], cvScalar(0, 255, 0), 4);
      cvLine(img, *pt[2], *pt[3], cvScalar(0, 255, 0), 4);
      cvLine(img, *pt[3], *pt[4], cvScalar(0, 255, 0), 4);
      cvLine(img, *pt[4], *pt[0], cvScalar(0, 255, 0), 4);
    }

    // get next contour
    contours = contours->h_next;

    ROS_INFO("HERE");
  }

  ROS_INFO("done");
  ROS_INFO("numTriangles = %d, numQuads = %d, numPentagons = %d\n", numTriangles, numQuads, numPentagons);

  // show img where shapes are marked
  cvNamedWindow("Tracked");
  cvShowImage("Tracked", img);

  cvWaitKey(0);

  cvDestroyAllWindows();
  cvReleaseMemStorage(&storage);
  cvReleaseImage(&img);
  cvReleaseImage(&imgGreyscale);

  ROS_INFO("Finished image processing.");
  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msgCorner, msgLandmark, msgRobot;

    std::stringstream ss;
    ss << "hello corner locations\nNumTriangles = " << numTriangles;
    msgCorner.data = ss.str();

    ss.str(std::string());
    ss << "hello landmark locations\nNumQuadrilaterals = " << numQuads;
    msgLandmark.data = ss.str();

    ss.str(std::string());
    ss << "hello robot locations\nNumPentagons = " << numPentagons;
    msgRobot.data = ss.str();

    ROS_INFO("%s", msgCorner.data.c_str());
    ROS_INFO("%s", msgLandmark.data.c_str());
    ROS_INFO("%s", msgRobot.data.c_str());

    corners_pub.publish(msgCorner);
    landmarks_pub.publish(msgLandmark);
    robots_pub.publish(msgRobot);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}

