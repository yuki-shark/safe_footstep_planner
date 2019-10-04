#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include "opencv_apps/Contour.h"
#include "opencv_apps/ContourArrayStamped.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <safe_footstep_planner/safe_footstep_util.h>
#include <safe_footstep_planner/PolygonArray.h>
#include <geometry_msgs/Polygon.h>

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
  ros::Publisher polygon_pub_;

  ros::Subscriber camera_info_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber contours_sub_;

  cv::Mat depth_image_;
  image_geometry::PinholeCameraModel model_;
  tf::TransformListener listener_;

  void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void DepthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void ContoursCallback(const opencv_apps::ContourArrayStamped::ConstPtr& msg);
};

ContoursConverter::ContoursConverter() : nh_(""), pnh_("~")
{
  img_pub_ = nh_.advertise<sensor_msgs::Image>("board_image", 1);
  steppable_region_pub_ = nh_.advertise<sensor_msgs::Image>("steppable_region_image", 1);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("contours_pointcloud", 1);
  polygon_pub_ = nh_.advertise<safe_footstep_planner::PolygonArray>("steppable_polygons", 1);
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
  // check if required topics are subscribed
  if (depth_image_.empty()) {
    return;
  }
  if (camera_info_.width < 0) {
    return;
  }

  std::vector<opencv_apps::Contour> contours;
  contours = msg->contours;

  double depth;

  double center_x = model_.cx();
  double center_y = model_.cy();

  double unit_scaling = 1;
  double constant_x = unit_scaling / model_.fx();
  double constant_y = unit_scaling / model_.fy();

  // pointcloud msg initialization
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);

  cloud_msg->header = msg->header;
  cloud_msg->header.frame_id = "/map";
  cloud_msg->height = 480;
  cloud_msg->width  = 640;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

  // tf initialization
  tf::StampedTransform transform;
  listener_.lookupTransform("/left_camera_optical_frame", "/map", ros::Time(0), transform);
  Eigen::Vector3f pos;
  safe_footstep_util::vectorTFToEigen(transform.getOrigin(), pos);
  Eigen::Matrix3f rot;
  safe_footstep_util::matrixTFToEigen(transform.getBasis(), rot);

  // convert pixel to coordinate
  safe_footstep_planner::PolygonArray polygon_array_msg;
  for (int i = 0; i < contours.size(); i++) {
    geometry_msgs::Polygon polygon;
    // std::cout << "contour : " << i << std::endl;
    for (opencv_apps::Point2D p : contours[i].points) {
      depth = depth_image_.at<float>(int(p.y), int(p.x));

      // transform coordinate
      geometry_msgs::Point32 original_point;
      original_point.x = (p.x - center_x) * depth * constant_x;
      original_point.y = (p.y - center_y) * depth * constant_y;
      original_point.z = depth;

      geometry_msgs::Point32 transformed_point;
      safe_footstep_util::transformPoint(original_point, rot, pos, transformed_point);

      // std::cout << "x : " << transformed_point.x << "  y : " << transformed_point.y << "  z : " << transformed_point.z << std::endl;

      *iter_x = transformed_point.x;
      *iter_y = transformed_point.y;
      *iter_z = transformed_point.z;

      polygon.points.push_back(transformed_point);

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    polygon_array_msg.polygons.push_back(polygon);
  }
  pointcloud_pub_.publish(cloud_msg);
  polygon_pub_.publish(polygon_array_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "contours_converter");
  ContoursConverter contours_converter;
  ros::spin();

  return 0;
}
