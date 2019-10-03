#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include "opencv_apps/FindContoursConfig.h"
#include "opencv_apps/Contour.h"
#include "opencv_apps/ContourArray.h"
#include "opencv_apps/ContourArrayStamped.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class ContoursConverter
{
public:
  ContoursConverter();
  ~ContoursConverter(){};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  opencv_apps::ContourArrayStamped contours_;
  sensor_msgs::CameraInfo camera_info_;
  ros::Publisher img_pub_;
  ros::Publisher steppable_region_pub_;
  ros::Publisher pointcloud_pub_;

  ros::Subscriber camera_info_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber contours_sub_;

  cv::Mat depth_image_;
  image_geometry::PinholeCameraModel model_;

  void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void DepthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void ContoursCallback(const opencv_apps::ContourArrayStamped::ConstPtr& msg);
};

ContoursConverter::ContoursConverter() : nh_(""), pnh_("~")
{
  img_pub_ = nh_.advertise<sensor_msgs::Image>("board_image", 1);
  steppable_region_pub_ = nh_.advertise<sensor_msgs::Image>("steppable_region_image", 1);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("contours_pointcloud", 1);
  camera_info_sub_ = nh_.subscribe("/multisense_local/left/camera_info", 1, &ContoursConverter::CameraInfoCallback, this);
  depth_image_sub_ = nh_.subscribe("/multisense_local/depth", 1, &ContoursConverter::DepthImageCallback, this);
  contours_sub_ = nh_.subscribe("/convex_full/hulls", 1, &ContoursConverter::ContoursCallback, this);
}

void ContoursConverter::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  camera_info_ = *msg;
  model_.fromCameraInfo(camera_info_);
}

void ContoursConverter::DepthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

  depth_image_ = cv_ptr->image;
}

void ContoursConverter::ContoursCallback(const opencv_apps::ContourArrayStamped::ConstPtr& msg) {
  if (depth_image_.empty()) {
    return;
  }

  std::cout << "depth_image_" << std::endl;

  if (camera_info_.width < 0) {
    return;
  }

  std::cout << "camera_info_" << std::endl;

  std::vector<opencv_apps::Contour> contours;
  contours = msg->contours;

  double depth;

  double center_x = model_.cx();
  double center_y = model_.cy();

  double unit_scaling = 1;
  double constant_x = unit_scaling / model_.fx();
  double constant_y = unit_scaling / model_.fy();

  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);

  cloud_msg->header = msg->header;
  cloud_msg->height = 480;
  cloud_msg->width  = 640;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

  for (int i = 0; i < contours.size(); i++) {
    std::cout << "contour : " << i << std::endl;
    for (opencv_apps::Point2D p : contours[i].points) {
      depth = depth_image_.at<float>(int(p.y), int(p.x));
      // depth = depth_image_.at<double>(int(p.x), int(p.y));
      // pz = depth_image_[int(p.x)][int(p.y)];
      std::cout << "x : " << int(p.x) << "  y : " << int(p.y) << "  z : " << depth << std::endl;
      *iter_x = (p.x - center_x) * depth * constant_x;
      *iter_y = (p.y - center_y) * depth * constant_y;
      *iter_z = depth;

      ++iter_x;
      ++iter_y;
      ++iter_z;

      // std::cout << "ix : " << iter_x << "  iy : " << iter_y << "  iz : " << iter_z << std::endl;
    }
  }

  // publish pointcloud for debug

  //hoge


  // std::cout << "Camera Info " << std::endl;

  // std::vector<std::vector<cv::Point> > contours;
  // std::vector<cv::Vec4i> hierarchy;

  // cv::Mat src_gray = cv_ptr->image;
  // cv::Mat img_and;
  // cv::bitwise_and(filled_convex_image_, src_gray, img_and);

  // sensor_msgs::Image::Ptr out_img =
  //   cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, img_and).toImageMsg();
  // steppable_region_pub_.publish(out_img);
  pointcloud_pub_.publish(cloud_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "contours_converter");
  ContoursConverter contours_converter;
  ros::spin();

  return 0;
}
