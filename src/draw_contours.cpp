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
  ros::Publisher img_pub_;
  ros::Publisher steppable_region_pub_;
  ros::Publisher msg_pub_;

  ros::Subscriber contours_sub_;
  ros::Subscriber board_hsv_sub_;
  ros::Subscriber region_hsv_sub_;

  cv::Mat filled_convex_image_;
  void ImageBoardCallback(const sensor_msgs::Image::ConstPtr& msg);
  void ImageRegionCallback(const sensor_msgs::Image::ConstPtr& msg);
};

DrawContours::DrawContours() : nh_(""), pnh_("~")
{
  img_pub_ = nh_.advertise<sensor_msgs::Image>("board_image", 1);
  steppable_region_pub_ = nh_.advertise<sensor_msgs::Image>("steppable_region_image", 1);
  msg_pub_ = nh_.advertise<opencv_apps::ContourArrayStamped>("hulls", 1);
  board_hsv_sub_ = nh_.subscribe("/board_erode_mask_image2/output", 1, &DrawContours::ImageBoardCallback, this);
  region_hsv_sub_ = nh_.subscribe("/left_erode_mask_image/output", 1, &DrawContours::ImageRegionCallback, this);
}

void DrawContours::ImageBoardCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::Mat src_gray = cv_ptr->image;

  // Messages
  opencv_apps::ContourArrayStamped contours_msg;
  contours_msg.header = msg->header;

  cv::blur(src_gray, src_gray, cv::Size(3, 3));

  /// Find contours
  cv::findContours(src_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Find the convex hull object for each contour
  std::vector<std::vector<cv::Point> > hull(contours.size());
  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
  }

  cv::Mat drawing = cv::Mat::zeros(src_gray.size(), CV_8UC1);
  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::Scalar color = cv::Scalar(255);
    cv::drawContours(drawing, hull, (int)i, color, -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

    opencv_apps::Contour contour_msg;
    // c++11 option required here
    for (const cv::Point& j : hull[i])
    {
      opencv_apps::Point2D point_msg;
      point_msg.x = j.x;
      point_msg.y = j.y;
      contour_msg.points.push_back(point_msg);
    }
    contours_msg.contours.push_back(contour_msg);
  }

  filled_convex_image_ = drawing;
  sensor_msgs::Image::Ptr out_img =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, drawing).toImageMsg();
  img_pub_.publish(out_img);
  msg_pub_.publish(contours_msg);

}

void DrawContours::ImageRegionCallback(const sensor_msgs::Image::ConstPtr& msg) {
  if (filled_convex_image_.empty()) {
    return;
  }
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::Mat src_gray = cv_ptr->image;
  cv::Mat img_and;
  cv::bitwise_and(filled_convex_image_, src_gray, img_and);

  sensor_msgs::Image::Ptr out_img =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, img_and).toImageMsg();
  steppable_region_pub_.publish(out_img);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher");
  DrawContours draw_contours;
  ros::spin();

  return 0;
}
