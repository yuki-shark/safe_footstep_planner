#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
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

class TargetHeightPublisher
{
public:
    TargetHeightPublisher();
    ~TargetHeightPublisher(){};

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
    void matrixTFToEigen(const tf::Matrix3x3 &t, Eigen::Matrix3f &e);
    void vectorTFToEigen(const tf::Vector3& t, Eigen::Vector3f& k);
    void calcFootRotFromNormal (Eigen::Matrix3f& foot_rot, const Eigen::Matrix3f& orig_rot, const Eigen::Vector3f& n);
    Eigen::Vector3f rpyFromRot(const Eigen::Matrix3f& m);
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher height_publisher_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber target_sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    tf::TransformListener listener_;
};

TargetHeightPublisher::TargetHeightPublisher() : nh_(""), pnh_("~")
{
    height_publisher_ = nh_.advertise<safe_footstep_planner::OnlineFootStep>("landing_height", 1);
    cloud_sub_ = nh_.subscribe("accumulated_heightmap_pointcloud_maprelative/output", 1, &TargetHeightPublisher::pointcloudCallback, this);
    target_sub_ = nh_.subscribe("landing_target", 1, &TargetHeightPublisher::targetCallback, this);
}

void TargetHeightPublisher::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_);
    // std::cout << "cloud size : " << cloud_->size() << std::endl;
}

void TargetHeightPublisher::targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg)
{
    double px = msg->x;
    double py = msg->y;
    double pz = msg->z;

    // transform
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
    vectorTFToEigen(transform.getOrigin(), cur_foot_pos);
    Eigen::Matrix3f tmp_cur_foot_rot, cur_foot_rot;
    matrixTFToEigen(transform.getBasis(), tmp_cur_foot_rot);
    calcFootRotFromNormal(cur_foot_rot, tmp_cur_foot_rot, ez);
    Eigen::Vector3f next_foot_pos;
    next_foot_pos = cur_foot_pos + cur_foot_rot * Eigen::Vector3f(px, py, pz);

    double threshold = 0.05;
    double cur_az = 0.0, next_az = 0.0;
    int count_cur = 0, count_next = 0;
    pcl::PointXYZ pp;
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    if (cloud_) {
        for (int i = 0; i < cloud_->size(); i++) {
            pp = cloud_->points[i];
            if (std::fabs(pp.x - next_foot_pos(0)) < threshold &&
                std::fabs(pp.y - next_foot_pos(1)) < threshold) {
                next_az += pp.z;
                count_next++;
                indices->indices.push_back(i);
            }
            if (std::fabs(pp.x - cur_foot_pos(0)) < threshold &&
                std::fabs(pp.y - cur_foot_pos(1)) < threshold) {
              cur_az += pp.z;
              count_cur++;
            }
        }
        // ROS_INFO("x: %f  y: %f  z: %f,  rel_pos x: %f  rel_pos y: %f, az/count: %f", landing_pos.getX(), landing_pos.getY(), landing_pos.getZ(), rel_landing_pos.getX(), rel_landing_pos.getY(), az / count);

        // publish point
        safe_footstep_planner::OnlineFootStep ps;
        std_msgs::Header header;
        header.frame_id = target_frame.substr(1, target_frame.length() - 1);
        ps.header = header;
        Eigen::Vector3f tmp_pos;
        cur_foot_pos(2) = cur_az / static_cast<double>(count_cur);
        next_foot_pos(2) = next_az / static_cast<double>(count_next);
        tmp_pos = cur_foot_rot.transpose() * (next_foot_pos - cur_foot_pos);
        ps.x = tmp_pos(0);
        ps.y = tmp_pos(1);
        ps.z = tmp_pos(2);
        ps.l_r = msg->l_r;

        // estimage plane by RANSAC
        int minimun_indices = 10;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setRadiusLimits(0.01, std::numeric_limits<double>::max ());
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setInputCloud(cloud_);
        //
        seg.setIndices(indices);
        seg.setMaxIterations(100);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            std::cerr <<  " no plane" << std::endl;
        }
        else if (inliers->indices.size() < minimun_indices) {
            std::cerr <<  " no enough inliners " << inliers->indices.size() <<  std::endl;
        }
        else {
            jsk_recognition_utils::Plane plane(coefficients->values);
              if (!plane.isSameDirection(ez)) {
                plane = plane.flip();
            }
            Eigen::Vector3f next_n = plane.getNormal();
            next_n = cur_foot_rot.transpose() * next_n; // cur_foot relative

            ps.nx =  next_n(0);
            ps.ny =  next_n(1);
            ps.nz =  next_n(2);
        }
        height_publisher_.publish(ps);
    }
}

void TargetHeightPublisher::matrixTFToEigen(const tf::Matrix3x3 &t, Eigen::Matrix3f &e)
{
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      e(i,j) = t[i][j];
}

void TargetHeightPublisher::vectorTFToEigen(const tf::Vector3& t, Eigen::Vector3f& k)
{
    k(0) = t[0];
    k(1) = t[1];
    k(2) = t[2];
}

Eigen::Vector3f TargetHeightPublisher::rpyFromRot(const Eigen::Matrix3f& m)
{
  double roll, pitch, yaw;

  if ((fabs(m(0,0))<fabs(m(2,0))) && (fabs(m(1,0))<fabs(m(2,0)))) {
    // cos(p) is nearly = 0
    double sp = -m(2,0);
    if (sp < -1.0) {
      sp = -1;
    } else if (sp > 1.0) {
      sp = 1;
    }
    pitch = asin(sp); // -pi/2< p < pi/2

    roll = atan2(sp*m(0,1)+m(1,2),  // -cp*cp*sr*cy
                 sp*m(0,2)-m(1,1)); // -cp*cp*cr*cy

    if (m(0,0)>0.0) { // cy > 0
      (roll < 0.0) ? (roll += M_PI) : (roll -= M_PI);
    }
    double sr=sin(roll), cr=cos(roll);
    if (sp > 0.0) {
      yaw = atan2(sr*m(1,1)+cr*m(1,2), //sy*sp
                  sr*m(0,1)+cr*m(0,2));//cy*sp
    } else {
      yaw = atan2(-sr*m(1,1)-cr*m(1,2),
                  -sr*m(0,1)-cr*m(0,2));
    }
  } else {
    yaw = atan2(m(1,0), m(0,0));
    const double sa = sin(yaw);
    const double ca = cos(yaw);
    pitch = atan2(-m(2,0), ca*m(0,0)+sa*m(1,0));
    roll = atan2(sa*m(0,2)-ca*m(1,2), -sa*m(0,1)+ca*m(1,1));
  }
  return Eigen::Vector3f(roll, pitch, yaw);
}

void TargetHeightPublisher::calcFootRotFromNormal (Eigen::Matrix3f& foot_rot, const Eigen::Matrix3f& orig_rot, const Eigen::Vector3f& n)
{
  Eigen::Vector3f ex = Eigen::Vector3f::UnitX();
  Eigen::Vector3f xv1(orig_rot * ex);
  xv1 = xv1 - xv1.dot(n) * n;
  xv1.normalize();
  Eigen::Vector3f yv1(n.cross(xv1));
  foot_rot(0,0) = xv1(0); foot_rot(1,0) = xv1(1); foot_rot(2,0) = xv1(2);
  foot_rot(0,1) = yv1(0); foot_rot(1,1) = yv1(1); foot_rot(2,1) = yv1(2);
  foot_rot(0,2) = n(0);  foot_rot(1,2) = n(1);  foot_rot(2,2) = n(2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_height_publisher");
    TargetHeightPublisher target_height_publisher;
    ros::spin();

    return 0;
}
