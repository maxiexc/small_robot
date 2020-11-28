#include "odometry.h"

namespace small_robot_llc
{
  Odometry::Odometry()
  {

  }

  void Odometry::Init()
  {

  }

  void Odometry::Update()
  {
    double mov_linear;
    double mov_angular;
    // Calculate Speed
    // Calculate movement
    mov_linear = 0.5*(r + l);
    mov_angular = (r - l)/track_width;
    // Add movement onto current pose
    pose_now.x += mov_linear * cos(mov_angular);
    pose_now.y += mov_linear * sin(mov_angular);
    pose_now.heading += mov_angular;

  }

  void Odometry::SetX(double x)
  {
    pose_now.x = x;
  }

  void Odometry::SetY(double y)
  {
    pose_now.y = y;
  }

  void Odometry::SetHeading(double heading)
  {
    pose_now.heading = heading;
  }

  double Odometry::GetX()
  {
    return pose_now.x;
  }

  double Odometry::GetY()
  {
    return pose_now.y;
  }

  double Odometry::GetHeading()
  {
    return pose_now.z;
  }
}