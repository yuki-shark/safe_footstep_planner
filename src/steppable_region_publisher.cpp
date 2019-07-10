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
#include <safe_footstep_planner/PolygonArray.h>
#include "safe_footstep_util.h"

class SteppableRegionPublisher
{
public:
  SteppableRegionPublisher();
  ~SteppableRegionPublisher(){};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  safe_footstep_planner::PolygonArray combined_meshes_;
  ros::Subscriber target_sub_;
  ros::Subscriber mesh_sub_;
  ros::Publisher region_publisher_;
  ros::Publisher combined_mesh_publisher_;
  tf::TransformListener listener_;
  void polygonarrayCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
  void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
  void meshCallback(const safe_footstep_planner::PolygonArray::ConstPtr& msg);
};

SteppableRegionPublisher::SteppableRegionPublisher() : nh_(""), pnh_("~")
{
  region_publisher_ = nh_.advertise<safe_footstep_planner::SteppableRegion>("steppable_region", 1);
  combined_mesh_publisher_ = nh_.advertise<safe_footstep_planner::PolygonArray>("combined_meshed_polygons", 1);
  target_sub_ = nh_.subscribe("landing_target", 1, &SteppableRegionPublisher::targetCallback, this);
  mesh_sub_ = nh_.subscribe("meshed_polygons", 1, &SteppableRegionPublisher::meshCallback, this);
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
  size_t convex_num(combined_meshes_.polygons.size());
  sr.polygons.resize(convex_num);
  for (size_t i = 0; i < convex_num; i++) {
    size_t vs_num(combined_meshes_.polygons[i].points.size());
    sr.polygons[i].polygon.points.resize(vs_num);
    for (size_t j = 0; j < vs_num; j++) {
      safe_footstep_util::transformPoint(combined_meshes_.polygons[i].points[j], cur_foot_rot, cur_foot_pos, sr.polygons[i].polygon.points[j]);
    }
  }

  std_msgs::Header header;
  header.frame_id = target_frame.substr(1, target_frame.length() - 1);
  sr.header = header;
  sr.l_r = msg->l_r;

  region_publisher_.publish(sr);
}

void SteppableRegionPublisher::meshCallback(const safe_footstep_planner::PolygonArray::ConstPtr& msg)
{
  std::vector<std::vector<Eigen::Vector3f> > meshes;
  std::vector<std::vector<Eigen::Vector3f> > combined_meshes;
  std::vector<std::vector<size_t> > combined_indices;

  // convert to Eigen::Vector3f
  size_t mesh_num(msg->polygons.size());
  meshes.resize(mesh_num);
  // std::cerr << "mesh_num : " << mesh_num << std::endl;
  std::vector<bool> is_combined(mesh_num, false);
  for (size_t i = 0; i < mesh_num; i++) {
    size_t vs_num(msg->polygons[i].points.size()); // must be 3 (triangle)
    meshes[i].resize(vs_num);
    for (size_t j = 0; j < vs_num; j++) {
      safe_footstep_util::pointsToEigen(msg->polygons[i].points[j], meshes[i][j]);
    }
  }

  // debug
  // size_t mesh_num(4);
  // std::vector<bool> is_combined(mesh_num, false);
  // meshes.resize(mesh_num);
  // meshes[0].push_back(Eigen::Vector3f(0, 0, 0));
  // meshes[0].push_back(Eigen::Vector3f(500, 0, 0));
  // meshes[0].push_back(Eigen::Vector3f(700, 500, 0));
  // meshes[1].push_back(Eigen::Vector3f(400, 800, 0));
  // meshes[1].push_back(Eigen::Vector3f(700, 500, 0));
  // meshes[1].push_back(Eigen::Vector3f(1000, 700, 0));
  // meshes[2].push_back(Eigen::Vector3f(700, 500, 0));
  // meshes[2].push_back(Eigen::Vector3f(400, 800, 0));
  // meshes[2].push_back(Eigen::Vector3f(0, 0, 0));
  // meshes[3].push_back(Eigen::Vector3f(1000, 700, 0));
  // meshes[3].push_back(Eigen::Vector3f(650, 850, 0));
  // meshes[3].push_back(Eigen::Vector3f(400, 800, 0));

  // combine meshes
  for (size_t i = 0; i < meshes.size(); i++) {
    std::vector<size_t> is_combined_indices;
    is_combined_indices.push_back(i);
    for (size_t j = i + 1; j < meshes.size(); j++) {
      std::vector<Eigen::Vector3f> inter_v;
      std::vector<Eigen::Vector3f> v1 = meshes[i], v2 = meshes[j];
      std::sort(v1.begin(), v1.end(), safe_footstep_util::compare_eigen3f);
      std::sort(v2.begin(), v2.end(), safe_footstep_util::compare_eigen3f);
      std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), inserter(inter_v, inter_v.end()), safe_footstep_util::compare_eigen3f);
      if (inter_v.size() == 2) { // adjacent mesh
        std::vector<Eigen::Vector3f> tmp_vs(v1), target_v, tmp_convex;
        std::set_difference(v2.begin(), v2.end(), v1.begin(), v1.end(), inserter(target_v, target_v.end()), safe_footstep_util::compare_eigen3f);
        std::copy(target_v.begin(), target_v.end(), std::back_inserter(tmp_vs));
        safe_footstep_util::calc_convex_hull(tmp_vs, tmp_convex);
        if (tmp_vs.size() == tmp_convex.size()) {
          meshes[i] = tmp_convex;
          meshes[j] = tmp_convex;
          is_combined[j] = true;
          is_combined_indices.push_back(j);
        }
      }
    }
    if (!is_combined[i]) {
      combined_meshes.push_back(meshes[i]);
      combined_indices.push_back(is_combined_indices);
    } else if (is_combined_indices.size() > 1) {
      for (size_t j = 0; j < combined_indices.size(); j++) {
        if (std::find(combined_indices[j].begin(), combined_indices[j].end(), i) != combined_indices[j].end()) {
          combined_meshes[j] = meshes[i];
          combined_indices[j] = is_combined_indices;
        }
      }
    }
    is_combined[i] = true;
  }

  // convert to safe_footstep_planner::PolygonArray
  size_t combined_mesh_num(combined_meshes.size());
  // std::cerr << "combined_mesh_num : " << combined_mesh_num << std::endl;
  combined_meshes_.polygons.resize(combined_mesh_num);
  for (size_t i = 0; i < combined_mesh_num; i++) {
    size_t vs_num(combined_meshes[i].size());
    combined_meshes_.polygons[i].points.resize(vs_num);
    for (size_t j = 0; j < vs_num; j++) {
      safe_footstep_util::eigenToPoints(combined_meshes[i][j], combined_meshes_.polygons[i].points[j]);
    }
  }

  combined_mesh_publisher_.publish(combined_meshes_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steppable_region_publisher");
  SteppableRegionPublisher steppable_region_publisher;
  ros::spin();

  return 0;
}

