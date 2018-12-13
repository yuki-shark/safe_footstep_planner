#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <cmath>

#define FOOT_SIZE 0.26

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    tf::TransformListener listener;
    ros::Subscriber info_sub_;
    sensor_msgs::CameraInfoConstPtr info_msg;

  public:
    ImageConverter()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("/label_image", 1,
                                   &ImageConverter::imageCb, this);
        info_sub_ = nh_.subscribe("/static_virtual_camera/camera_info", 1,
                                   &ImageConverter::infoCb, this);
    }

    ~ImageConverter()
    {

    }

    void infoCb(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        info_msg = msg;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_32SC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        std::cout << "rows : " << cv_ptr->image.rows << std::endl;
        std::cout << "cols : " << cv_ptr->image.cols << std::endl;

        tf::StampedTransform transform;
        try {
            listener.lookupTransform("map", "static_virtual_camera", ros::Time(0), transform);

            tf::Vector3 original_coords;
            original_coords.setValue(0.5, 0.2, 0.0);
            std::cout << "original coords : " << "x : 0.5  y : 0.2  z : 0.0" << std::endl;

            tf::Vector3 translation_vector = transform.getOrigin();
            tf::Matrix3x3 rotation_matrix = transform.getBasis();
            tf::Vector3 target_coords;

            target_coords = rotation_matrix.inverse() * (original_coords - translation_vector);
            std::cout << "target coords : ";
            std::cout << "  x : " << target_coords.x();
            std::cout << "  y : " << target_coords.y();
            std::cout << "  z : " << target_coords.z();
            std::cout << std::endl;

            // calculate fov
            float fx = info_msg->K[0];
            float fy = info_msg->K[4];
            int image_width  = info_msg->width;
            int image_height = info_msg->height;

            double fovx, fovy;
            fovx = 2 * std::atan(image_width /(2*fx));
            fovy = 2 * std::atan(image_height/(2*fy));

            // convert target coords to image pixel
            double fov_width  = 2 * target_coords.z() * std::tan(fovx/2);
            double fov_height = 2 * target_coords.z() * std::tan(fovy/2);

            int center_u = (target_coords.x() + fov_width /2) / fov_width  * image_width;
            int center_v = (target_coords.y() + fov_height/2) / fov_height * image_height;

            std::cout << "target pixel : ";
            std::cout << " center_u : " << center_u;
            std::cout << " center_v : " << center_v;
            std::cout << std::endl;

            // std::cout << "target label" << static_cast<int>(cv_ptr->image.data[center_u*image_width+center_v]) << std::endl;

            // get mean of footfold safety
            int radius = (FOOT_SIZE / fov_width * image_width) / 2;
            std::cout << "radius : " << radius << std::endl;
            int left_u   = std::max(center_u - radius, 0);
            int right_u  = std::min(center_u + radius + 1, image_width);
            int top_v    = std::max(center_u - radius, 0);
            int bottom_v = std::min(center_u + radius + 1, image_width);

            int sum = 0;
            for (int i=left_u; i<right_u; i++) {
                for (int j=top_v; j<bottom_v; j++) {
                    sum += cv_ptr->image.data[i*image_width + j];
                }
            }

            double safety_cost = 1.0 * sum / ((right_u - left_u) * (bottom_v - top_v));

            std::cout << "safety cost : " << safety_cost << std::endl;
            std::cout << "=============================================================" << std::endl;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
