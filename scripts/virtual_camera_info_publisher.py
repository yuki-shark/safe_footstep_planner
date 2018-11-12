#!/usr/bin/env python

# import cv2
# try:
#     from scipy.misc import face
#     img = face()[:, :, ::-1]
# except ImportError:
#     import numpy as np
#     from scipy.misc import lena
#     img = cv2.cvtColor(lena().astype(np.uint8), cv2.COLOR_GRAY2BGR)

# import cv_bridge
import rospy
# from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


def timer_cb(event):
    # imgmsg.header.stamp = rospy.Time.now()
    # info_msg.header.stamp = imgmsg.header.stamp
    info_msg.header.stamp = rospy.Time.now()
    # pub_img.publish(imgmsg)
    pub_info.publish(info_msg)


if __name__ == '__main__':
    rospy.init_node('static_virtual_camera')

    # pub_img = rospy.Publisher('~image_color', Image, queue_size=1)
    pub_info = rospy.Publisher('~camera_info', CameraInfo, queue_size=1)

    # bridge = cv_bridge.CvBridge()
    # imgmsg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    # imgmsg.header.frame_id = 'static_virtual_camera'

    info_msg = CameraInfo()
    # info_msg.header.frame_id = imgmsg.header.frame_id
    info_msg.header.frame_id = 'static_virtual_camera'

    # height, width = img.shape[:2]
    # info_msg.height = height
    # info_msg.width = width
    info_msg.height = 480
    info_msg.width = 640

    rospy.Timer(rospy.Duration(1.0/30), timer_cb)
    rospy.spin()
