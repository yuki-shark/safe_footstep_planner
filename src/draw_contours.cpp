#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include "opencv_apps/FindContoursConfig.h"
#include "opencv_apps/Contour.h"
#include "opencv_apps/ContourArray.h"
#include "opencv_apps/ContourArrayStamped.h"

#include <sensor_msgs/Image.h>

class DrawContours
{
public:
  DrawContours();
  ~DrawContours(){};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  opencv_apps::ContourArrayStamped contours_;
  sensor_msgs::Image image_;
  // sensor_msgs::CvBridge bridge_;
  // opencv_apps::ContourArrayStamped contours_;
  ros::Subscriber contours_sub_;
  ros::Publisher image_publisher_;
  void ContourArrayCallback(const opencv_apps::ContourArrayStamped::ConstPtr& msg);
  // void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
};

DrawContours::DrawContours() : nh_(""), pnh_("~")
{
  image_publisher_ = nh_.advertise<sensor_msgs::Image>("board_image", 1);
  // contours_sub_ = nh_.subscribe("/board_convex_full/hulls", 1, &DrawContours::ContourArrayCallback, this);
  board_hsv_sub_ = nh_.subscribe("/board_hsv_color_filter/image", 1, &DrawContours::ContourArrayCallback, this);
}

// void DrawContours::ContourArrayCallback(const opencv_apps::ContourArrayStamped::ConstPtr& msg)
// {
//   // contours_ = msg->contours;
//   // cv::Mat img = cv::Mat::zeros(msg->contours.size(), CV_8UC1);
//   cv_bridge::CvImage outimg;
//   // // cv::drawContours(img, msg->contours, -1, 255, -1);
//   // for (opencv_apps::Contour contour : msg->contours) {
//   //   cv::drawContours(img, msg->contours, -1, 255, -1);
//   // }

//   cv::Mat drawing = cv::Mat::zeros(544, 1024, CV_8UC1);
//   for (size_t i = 0; i < msg->contours.size(); i++)
//   {
//     cv::Scalar color = cv::Scalar(255);
//     // cv::drawContours(drawing, msg->contours, (int)i, color, 2, 8, vector<int>(0,4), 0, cv::Point());
//     // cv::drawContours(drawing, msg->contours, i, cv::Scalar(255, 0, 0), -1);
//     cv::drawContours(drawing, msg->contours, (int)i, color, -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
//     // cv::drawContours(drawing, msg->contours, (int)i, color, -1);
//   }
//   outimg.header = msg->header;
//   outimg.encoding = sensor_msgs::image_encodings::MONO8;
//   outimg.image = drawing;

//   image_publisher_.publish(outimg);
// }

void DrawContours::ContourArrayCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

  int max_thresh = 255;
  cv::Mat threshold_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::threshold(
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher");
  DrawContours draw_contours;
  ros::spin();

  return 0;
}
