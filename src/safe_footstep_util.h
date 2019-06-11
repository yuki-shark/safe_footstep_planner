#ifndef SAFE_UTIL_H
#define SAFE_UTIL_H

// #include <ros/ros.h>
// #include <std_msgs/Header.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/PointStamped.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl/PointIndices.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <tf/transform_listener.h>
// #include <jsk_recognition_utils/pcl_conversion_util.h>
// #include <jsk_recognition_utils/geo_util.h>
// #include <safe_footstep_planner/OnlineFootStep.h>
// #include "safe_footstep_util.h"

namespace safe_footstep_util
{
  void matrixTFToEigen (const tf::Matrix3x3 &t, Eigen::Matrix3f &e)
  {
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        e(i,j) = t[i][j];
  }

  void vectorTFToEigen (const tf::Vector3& t, Eigen::Vector3f& k)
  {
    k(0) = t[0];
    k(1) = t[1];
    k(2) = t[2];
  }

  void transformPoint (const geometry_msgs::Point32& t, const Eigen::Matrix3f& rot, const Eigen::Vector3f& pos, geometry_msgs::Point32& k)
  {
    Eigen::Vector3f tmp_pos = rot.transpose() * (Eigen::Vector3f(t.x, t.y, t.z) - pos);
    k.x = tmp_pos(0);
    k.y = tmp_pos(1);
    k.z = tmp_pos(2);
  }

  Eigen::Vector3f rpyFromRot (const Eigen::Matrix3f& m)
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

  void calcFootRotFromNormal (Eigen::Matrix3f& foot_rot, const Eigen::Matrix3f& orig_rot, const Eigen::Vector3f& n)
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
}

#endif // SAFE_UTIL_H
