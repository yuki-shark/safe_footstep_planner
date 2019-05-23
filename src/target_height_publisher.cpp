#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <tf/transform_listener.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <safe_footstep_planner/TargetFoothold.h>

class TargetHeightPublisher
{
public:
    TargetHeightPublisher();
    ~TargetHeightPublisher(){};

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void targetCallback(const safe_footstep_planner::TargetFoothold::ConstPtr& msg);
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
    height_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("height", 1);
    cloud_sub_ = nh_.subscribe("accumulated_heightmap_pointcloud/output", 10, &TargetHeightPublisher::pointcloudCallback, this);
    target_sub_ = nh_.subscribe("target", 10, &TargetHeightPublisher::targetCallback, this);
}

void TargetHeightPublisher::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_);
    // std::cout << "cloud size : " << cloud_->size() << std::endl;
}

void TargetHeightPublisher::targetCallback(const safe_footstep_planner::TargetFoothold::ConstPtr& msg)
{
    double x = msg->target.x;
    double y = msg->target.y;

    // transform
    std::string target_frame;
    if (msg->leg) {
        target_frame = "/lleg_end_coords";
    }
    else {
        target_frame = "/rleg_end_coords";
    }

    tf::StampedTransform transform;
    listener_.lookupTransform("/BODY_on_map", target_frame, ros::Time(0), transform);
    tf::Vector3 original_coords;
    original_coords.setValue(x, y, 0.0);
    tf::Vector3 translation_vector = transform.getOrigin();
    tf::Matrix3x3 rotation_matrix = transform.getBasis();
    tf::Vector3 target_coords;
    target_coords = rotation_matrix.inverse() * (original_coords - translation_vector);

    // ROS_INFO("x: %f  y: %f  z: %f", target_coords.getX(), target_coords.getY(), target_coords.getZ());

    double threshold = 0.01;
    double az = 0.0;
    int count = 0;
    pcl::PointXYZ pp;

    if (cloud_) {
        for (int i = 0; i < cloud_->size(); i++) {
            pp = cloud_->points[i];
            if (std::abs(pp.x - target_coords.getX()) < threshold &&
                std::abs(pp.y - target_coords.getY()) < threshold) {
                az += pp.z;
                count++;
            }
        }
        // std::cout << "frame : " << target_frame << std::endl;
        // std::cout << "z :  " << az / count + target_coords.getZ() << std::endl;

        // publish point
        geometry_msgs::PointStamped ps;
        std_msgs::Header header;
        header.frame_id = target_frame.substr(1, target_frame.length() - 1);
        ps.header = header;
        ps.point.x = x;
        ps.point.y = y;
        ps.point.z = az / count + target_coords.getZ();
        height_publisher_.publish(ps);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_height_publisher");
    TargetHeightPublisher target_height_publisher;
    ros::spin();

    return 0;
}
