#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <tf/transform_listener.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <safe_footstep_planner/OnlineFootStep.h>

class TargetHeightPublisher
{
public:
    TargetHeightPublisher();
    ~TargetHeightPublisher(){};

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void targetCallback(const safe_footstep_planner::OnlineFootStep::ConstPtr& msg);
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
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;

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
    tf::Vector3 landing_pos; // suppoting foot relative
    landing_pos.setValue(x, y, z);
    tf::Vector3 translation_vector = transform.getOrigin();
    tf::Matrix3x3 rotation_matrix = transform.getBasis();
    tf::Vector3 rel_landing_pos; // map relative
    rel_landing_pos = translation_vector + rotation_matrix * landing_pos;
    std::cerr << "******************* target_frame : " << target_frame << std::endl;
    std::cerr << "translation_vector: " << translation_vector.getX() << " , " << translation_vector.getY() << " , " << translation_vector.getZ() << std::endl;

    double threshold = 0.03;
    double az = 0.0, az2 = 0.0;
    int count = 0, count2 = 0;
    pcl::PointXYZ pp;

    if (cloud_) {
        for (int i = 0; i < cloud_->size(); i++) {
            pp = cloud_->points[i];
            if (std::fabs(pp.x - rel_landing_pos.getX()) < threshold &&
                std::fabs(pp.y - rel_landing_pos.getY()) < threshold) {
                az += pp.z;
                count++;
            }
            if (std::fabs(pp.x - translation_vector.getX()) < threshold &&
                std::fabs(pp.y - translation_vector.getY()) < threshold) {
              az2 += pp.z;
              count2++;
            }
        }
        // ROS_INFO("x: %f  y: %f  z: %f,  rel_pos x: %f  rel_pos y: %f, az/count: %f", landing_pos.getX(), landing_pos.getY(), landing_pos.getZ(), rel_landing_pos.getX(), rel_landing_pos.getY(), az / count);

        // publish point
        safe_footstep_planner::OnlineFootStep ps;
        std_msgs::Header header;
        header.frame_id = target_frame.substr(1, target_frame.length() - 1);
        ps.header = header;
        tf::Vector3 tmp_pos;
        rel_landing_pos.setZ(az / static_cast<double>(count));
        translation_vector.setZ(az2 / static_cast<double>(count2));
        tmp_pos = rotation_matrix.transpose() * (rel_landing_pos - translation_vector);
        ps.x = tmp_pos.getX();
        ps.y = tmp_pos.getY();
        ps.z = tmp_pos.getZ();
        ps.l_r = msg->l_r;
        height_publisher_.publish(ps);
        std::cerr << "foot pos : "<< landing_pos.getX() << ", " << landing_pos.getY() << ", " << landing_pos.getZ() << std::endl;
        std::cerr << "map pos swg: "<< rel_landing_pos.getX() << ", " << rel_landing_pos.getY() << ", " << rel_landing_pos.getZ() << std::endl;
        std::cerr << "map pos sup: "<< translation_vector.getX() << ", " << translation_vector.getY() << ", " << translation_vector.getZ() << std::endl;
        std::cerr << "height : "<< ps.x << ", " << ps.y << ", " << ps.z << ", " << ps.l_r << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_height_publisher");
    TargetHeightPublisher target_height_publisher;
    ros::spin();

    return 0;
}
