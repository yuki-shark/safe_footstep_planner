#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_listener.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_recognition_utils/geo_util.h>
#include <safe_footstep_planner/OnlineFootStep.h>
#include <safe_footstep_planner/SteppableRegion.h>
#include "safe_footstep_util.h"

class SteppableRegionPublisher
{
public:
  SteppableRegionPublisher();
  ~SteppableRegionPublisher(){};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  jsk_recognition_msgs::PolygonArray::ConstPtr polygons_;
  ros::Subscriber polygon_sub_;
  ros::Subscriber target_sub_;
  ros::Publisher region_publisher_;
  tf::TransformListener listener_;
  void polygonarrayCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
  void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
};

SteppableRegionPublisher::SteppableRegionPublisher() : nh_(""), pnh_("~")
{
  region_publisher_ = nh_.advertise<safe_footstep_planner::SteppableRegion>("steppable_region", 1);
  polygon_sub_ = nh_.subscribe("multi_plane_estimate/output_polygon", 1, &SteppableRegionPublisher::polygonarrayCallback, this);
  target_sub_ = nh_.subscribe("landing_target", 1, &SteppableRegionPublisher::targetCallback, this);
}

void SteppableRegionPublisher::polygonarrayCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
{
  polygons_ = msg;
}

void SteppableRegionPublisher::targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg)
{
  safe_footstep_planner::SteppableRegion sr;
  std::string target_frame;
  if (msg->l_r) {
    target_frame = "/lleg_end_coords";
  }
  else {
    target_frame = "/rleg_end_coords";
  }

  tf::StampedTransform transform;
  listener_.lookupTransform("/map", target_frame, ros::Time(0), transform); // map relative to target_frame
  Eigen::Vector3f cur_foot_pos, ez(Eigen::Vector3f::UnitZ());
  safe_footstep_util::vectorTFToEigen(transform.getOrigin(), cur_foot_pos);
  Eigen::Matrix3f tmp_cur_foot_rot, cur_foot_rot;
  safe_footstep_util::matrixTFToEigen(transform.getBasis(), tmp_cur_foot_rot);
  safe_footstep_util::calcFootRotFromNormal(cur_foot_rot, tmp_cur_foot_rot, ez);

  // convert to polygon relative to leg_end_coords
  size_t convex_num(polygons_->polygons.size());
  sr.polygons.resize(convex_num);
  for (size_t i = 0; i < convex_num; i++) {
    size_t vs_num(polygons_->polygons[i].polygon.points.size());
    sr.polygons[i].polygon.points.resize(vs_num);
    for (size_t j = 0; j < vs_num; j++) {
      safe_footstep_util::transformPoint(polygons_->polygons[i].polygon.points[j], cur_foot_rot, cur_foot_pos, sr.polygons[i].polygon.points[j]);
    }
  }
  sr.polygons = polygons_->polygons;

  std_msgs::Header header;
  header.frame_id = target_frame.substr(1, target_frame.length() - 1);
  sr.header = header;
  sr.l_r = msg->l_r;

  region_publisher_.publish(sr);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steppable_region_publisher");
  SteppableRegionPublisher steppable_region_publisher;
  ros::spin();

  return 0;
}

