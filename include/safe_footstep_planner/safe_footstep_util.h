#ifndef SAFE_UTIL_H
#define SAFE_UTIL_H

#define deg2rad(x) ((x)*M_PI/180)
#define rad2deg(rad) (rad*180/M_PI)
#define eps_eq(a, b, c) (fabs((a)-(b)) <= c)

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

  void pointsToEigen (const geometry_msgs::Point32& t, Eigen::Vector3f& k)
  {
    k(0) = t.x;
    k(1) = t.y;
    k(2) = t.z;
  }

  void eigenToPoints (const Eigen::Vector3f& t, geometry_msgs::Point32& k)
  {
    k.x = t(0);
    k.y = t(1);
    k.z = t(2);
  }

  void transformPoint (const geometry_msgs::Point32& t, const Eigen::Matrix3f& rot, const Eigen::Vector3f& pos, geometry_msgs::Point32& k)
  {
    Eigen::Vector3f tmp_pos = rot.transpose() * (Eigen::Vector3f(t.x, t.y, t.z) - pos);
    k.x = tmp_pos(0);
    k.y = tmp_pos(1);
    k.z = tmp_pos(2);
  }

  void convertTFToPose(const tf::Transform& tf, geometry_msgs::Pose& pose){
    pose.orientation.x = tf.getRotation().getAxis().x();
    pose.orientation.y = tf.getRotation().getAxis().y();
    pose.orientation.z = tf.getRotation().getAxis().z();
    pose.orientation.w = tf.getRotation().getAxis().w();
    pose.position.x = tf.getOrigin().getX();
    pose.position.y = tf.getOrigin().getY();
    pose.position.z = tf.getOrigin().getZ();
  }

  void convertPoseToTF(const geometry_msgs::Pose& pose, tf::Transform& tf){
    tf.getRotation().getAxis().setX(pose.orientation.x);
    tf.getRotation().getAxis().setY(pose.orientation.y);
    tf.getRotation().getAxis().setZ(pose.orientation.z);
    tf.getRotation().getAxis().setW(pose.orientation.w);
    tf.getOrigin().setX(pose.position.x);
    tf.getOrigin().setY(pose.position.y);
    tf.getOrigin().setZ(pose.position.z);
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
    Eigen::Vector3f en = n / n.norm();
    Eigen::Vector3f ex = Eigen::Vector3f::UnitX();
    Eigen::Vector3f xv1(orig_rot * ex);
    xv1 = xv1 - xv1.dot(en) * en;
    xv1.normalize();
    Eigen::Vector3f yv1(en.cross(xv1));
    foot_rot(0,0) = xv1(0); foot_rot(1,0) = xv1(1); foot_rot(2,0) = xv1(2);
    foot_rot(0,1) = yv1(0); foot_rot(1,1) = yv1(1); foot_rot(2,1) = yv1(2);
    foot_rot(0,2) = en(0);  foot_rot(1,2) = en(1);  foot_rot(2,2) = en(2);
  }

  bool compare_eigen3f(const Eigen::Vector3f& lv, const Eigen::Vector3f& rv)
  {
    return (lv(0) < rv(0)) || (lv(0) == rv(0) && lv(1) < rv(1)) || (lv(0) == rv(0) && lv(1) == rv(1) && lv(2) < rv(2));
  }

  double calc_cross_product(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& o)
  {
    return (a(0) - o(0)) * (b(1) - o(1)) - (a(1) - o(1)) * (b(0) - o(0));
  }

  void calc_convex_hull (std::vector<Eigen::Vector3f>& vs, std::vector<Eigen::Vector3f>& ch)
  {
    int n_vs = vs.size(), n_ch = 0;
    ch.resize(2*n_vs);
    std::sort(vs.begin(), vs.end(), compare_eigen3f);
    for (int i = 0; i < n_vs; ch[n_ch++] = vs[i++])
      while (n_ch >= 2 && calc_cross_product(ch[n_ch-1], vs[i], ch[n_ch-2]) <= 0) n_ch--;
    for (int i = n_vs-2, j = n_ch+1; i >= 0; ch[n_ch++] = vs[i--])
      while (n_ch >= j && calc_cross_product(ch[n_ch-1], vs[i], ch[n_ch-2]) <= 0) n_ch--;
    ch.resize(n_ch-1);
  }
}

#endif // SAFE_UTIL_H
