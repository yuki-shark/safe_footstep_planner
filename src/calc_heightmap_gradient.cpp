#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include<opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <image_geometry/pinhole_camera_model.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
// #include <geometry_msgs/PointStamped.h>
// #include <tf/transform_listener.h>

// #include <jsk_recognition_utils/pcl_conversion_util.h>
// #include <safe_footstep_planner/safe_footstep_util.h>

class HeightmapGradient
{
public:
  HeightmapGradient();
  ~HeightmapGradient(){};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // opencv_apps::ContourArrayStamped contours_;
  // sensor_msgs::CameraInfo camera_info_;
  // ros::Publisher img_pub_;
  // ros::Publisher steppable_region_pub_;
  // ros::Publisher pointcloud_pub_;
  // ros::Publisher polygon_pub_;

  // ros::Subscriber camera_info_sub_;
  ros::Subscriber heightmap_image_sub_;
  // ros::Subscriber contours_sub_;

  cv::Mat heightmap_;
  // image_geometry::PinholeCameraModel model_;
  // tf::TransformListener listener_;

  void HeightmapImageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

HeightmapGradient::HeightmapGradient() : nh_(""), pnh_("~")
{
  // img_pub_ = nh_.advertise<sensor_msgs::Image>("", 1);
  // steppable_region_pub_ = nh_.advertise<sensor_msgs::Image>("steppable_region_image", 1);
  // pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("contours_pointcloud", 1);
  // polygon_pub_ = nh_.advertise<safe_footstep_planner::PolygonArray>("steppable_polygons", 1);

  // heightmap_image_sub_ = nh_.subscribe("/heightmap_converter/output", 1, &HeightmapGradient::HeightmapImageCallback, this);
  heightmap_image_sub_ = nh_.subscribe("/accumulated_heightmap/output", 1, &HeightmapGradient::HeightmapImageCallback, this);
  // contours_sub_ = nh_.subscribe("/convex_full/hulls", 1, &HeightmapGradient::ContoursCallback, this);
}


void HeightmapGradient::HeightmapImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC2);

  heightmap_ = cv_ptr->image;
  std::vector<cv::Mat> planes;
  cv::split(heightmap_, planes);

  int height = msg->height;
  int width = msg->width;

  cv::Mat blur_img(height, width, CV_32FC1);
  cv::Mat sobelx_img(height, width, CV_32FC1);
  cv::Mat sobely_img(height, width, CV_32FC1);
  cv::Mat sobelxy_img(height, width, CV_32FC1);
  // cv::Mat lap_img(height, width, CV_32FC1);
  // blur(planes[0], blur_img, cv::Size(5,5));
  medianBlur(planes[0], blur_img, 5);
  // lap_img = cvCreateImage(cvGetSize(heightmap_), CV_32F, 1);
  // std::cerr << CV_32FC1 << std::endl;
  // Laplacian(heightmap_, lap_img, CV_32FC2, 1);
  // Laplacian(planes[0], lap_img, CV_32FC1, 1);
  Sobel(blur_img, sobelx_img, CV_32FC1, 1, 0, 27);
  Sobel(blur_img, sobely_img, CV_32FC1, 0, 1, 27);
  cv::pow(sobelx_img, 2, sobelx_img);
  cv::pow(sobely_img, 2, sobely_img);
  cv::add(sobelx_img, sobely_img, sobelxy_img);
  cv::sqrt(sobelxy_img, sobelxy_img);

  // Laplacian(blur_img, lap_img, CV_32FC1, 31);
  double minVal, maxVal;
  // cv::minMaxLoc(sobelxy_img, &minVal, &maxVal, NULL, NULL);
  // std::cout << "max : " << maxVal << std::endl;
  // std::cout << "min : " << minVal << std::endl;
  cv::divide(10.0, sobelxy_img, sobelxy_img);

  cv::imshow("Src", planes[0]);
  cv::imshow("Blur", blur_img);
  // cv::imshow("Lap", lap_img);
  cv::imshow("Sobel", sobelxy_img);

  // cv::imwrite("/tmp/src.jpg", planes[0]);
  // cv::imwrite("/tmp/blur.jpg", blur_img);
  // cv::imwrite("/tmp/lap.jpg", lap_img);

  cv::waitKey(0);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "heightmap_gradient");
  HeightmapGradient heightmap_gradient;
  ros::spin();

  return 0;
}
