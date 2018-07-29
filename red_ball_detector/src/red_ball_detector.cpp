#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

#include "std_msgs/Int16.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher ball_num_pub_;
  ros::Publisher ball_one_image_x_;
  std_msgs::Int16 balls;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed
    image_sub_ = it_.subscribe("/mybot/camera1/image_raw", 1, &ImageConverter::imageCb, this);
    ball_num_pub_ = nh_.advertise<std_msgs::Int16>("/red_ball_detector/num_balls", 1000);
    ball_one_image_x_ = nh_.advertise<std_msgs::Int16>("/red_ball_detector/ball_one_image_x", 1000);     
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //convert image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //set src to converted image
    Mat src = cv_ptr->image;

    // Convert input image to HSV
    cv::Mat hsv_image;
    cv::cvtColor(src, hsv_image, cv::COLOR_BGR2HSV);
 
    // Threshold the HSV image, keep only the red pixels
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
    
    // Combine the above two images
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    // Reduce the noise so we avoid false circle detection
    GaussianBlur( red_hue_image, red_hue_image, Size(9, 9), 2, 2 );

    vector<Vec3f> circles;

    // Apply the Hough Transform to find the circles
    HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 30, 0.2, 0.2);

    //print out number of red balls detected
    cout << "Found " << circles.size() << " ball(s)." << endl;
	
    if(circles.size() > 0) {
	balls.data = circles.size();
	ball_num_pub_.publish(balls);
   	balls.data = circles[0][0];
   	ball_one_image_x_.publish(balls);
    }
    else {
	balls.data = circles.size();
	ball_num_pub_.publish(balls);
	balls.data = -1;
	ball_one_image_x_.publish(balls);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_ball_detector");
  ImageConverter ic;
  ros::spin();
  return 0;
}
