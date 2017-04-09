#ifndef COMMON_H
#define COMMON_H

#include <cmath>

struct Twist {
  double lin_x, lin_y, lin_z;
  double ang_x, ang_y, ang_z;
};

struct Pose {
  double x, y, z;
  double quat_x, quat_y, quat_z, quat_w;
  double covariance[36];
  Twist velocity;
};

class Common
{
public:
  Common();
  void quatToEuler(double q_w, double q_x, double q_y, double q_z, double& yaw, double& pitch, double& roll);
  void setQuadPose(Pose p);
  Pose getQuadPose();
  void setgBot0Pose(Pose p);
  Pose getgBot0Pose();
  void setgBot1Pose(Pose p);
  Pose getgBot1Pose();
  void setgBot5Pose(Pose p);
  Pose getgBot5Pose();
protected:
  Pose quadPose, gBot0Pose, gBot1Pose, gBot5Pose;
};

#endif // COMMON_H
