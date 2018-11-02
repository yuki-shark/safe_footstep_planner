#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_recognition_msgs/Rect.h>
#include <std_msgs/Header.h>

#include <sstream>
// #include <copy>

class MaskImage
{
public:
    MaskImage();
    ~MaskImage(){};

private:
    void cameraInfoCallback(const sensor_msgs::CameraInfo camera_info);
    void rectPublisher(const std_msgs::Header header);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher rect_publisher_;
    ros::Subscriber camera_info_subscriber_;
};


MaskImage::MaskImage() : nh_(""), private_nh_("~")
{
    rect_publisher_ = nh_.advertise<jsk_recognition_msgs::RectArray>("mask_image_to_rect/output", 1);
    camera_info_subscriber_ = nh_.subscribe("zed/left/camera_info_raw", 1, &MaskImage::cameraInfoCallback, this);
}


void MaskImage::cameraInfoCallback(const sensor_msgs::CameraInfo camera_info)
{
    rectPublisher(camera_info.header);
}

void MaskImage::rectPublisher(const std_msgs::Header header)
{
    jsk_recognition_msgs::RectArray msg;
    msg.header = header;
    jsk_recognition_msgs::Rect rect;
    private_nh_.param("offset_x", rect.x,      0);
    private_nh_.param("offset_y", rect.y,      0);
    private_nh_.param("width",    rect.width,  1);
    private_nh_.param("height",   rect.height, 1);
    msg.rects.push_back(rect) ;
    rect_publisher_.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rect_publisher");
    MaskImage rect_publisher_node;
    ros::spin();

    return 0;
}
