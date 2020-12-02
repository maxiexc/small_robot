#include "odometry.h"
#include <cmath>

//namespace small_robot_llc
//{
  Odometry::Odometry(const double track_width, const double enc_count_per_meter)
  {
    track_width_ = track_width;
    enc_count_per_meter_ = enc_count_per_meter;
    enc_l_last_ = 0.0;
    enc_r_last_ = 0.0;
  }

  void Odometry::Init(const ros::Time &time)
  {
    time_last_ = time_now_ = time;
  }

  bool Odometry::Update(const double enc_l, const double enc_r, const ros::Time &time_now)
  {
    double mov_l;
    double mov_r;
    double mov_linear;
    double mov_angular;
    
    if(enc_count_per_meter_ < 0.01)
    {
      return true;
    }
    
    //update movement
    mov_l = (enc_l - enc_l_last_) / enc_count_per_meter_;
    mov_r = (enc_r - enc_r_last_) / enc_count_per_meter_;
    enc_l_last_ = enc_l;
    enc_r_last_ = enc_r;

    // Update time
    time_last_ = time_now_;
    time_now_ = time_now;    
    const  double dt = (time_now_ - time_last_ ).toSec();

    // Calculate movement
    if(track_width_ < 0.001)
    {
      return true;
    }

    mov_linear = 0.5*(mov_r + mov_l);
    mov_angular = (mov_r - mov_l)/track_width_;
    
    // Add movement onto current pose
    pose_last_ = pose_now_;
    pose_now_.x += mov_linear * cos(pose_now_.heading);
    pose_now_.y += mov_linear * sin(pose_now_.heading);
    pose_now_.heading += mov_angular;

    // Calculate Speed
    if(dt < 0.0001)
    {
      return true;
    }

    linear_ = (mov_linear/dt);
    angular_ = (mov_angular/dt);
    linear_x_ = (linear_ * (-1.0) * sin(pose_now_.heading));
    linear_y_ = (linear_ * (-1.0) * cos(pose_now_.heading));
    return false;    
  }

  void Odometry::SetX(double x)
  {
    pose_now_.x = x;
  }

  void Odometry::SetY(double y)
  {
    pose_now_.y = y;
  }

  void Odometry::SetHeading(double heading)
  {
    pose_now_.heading = heading;
  }

  double Odometry::GetX()
  {
    return pose_now_.x;
  }

  double Odometry::GetY()
  {
    return pose_now_.y;
  }

  double Odometry::GetHeading()
  {
    return pose_now_.heading;
  }

  double Odometry::GetVx()
  {
    return linear_x_;

  }
  double Odometry::GetVy()
  {
    return linear_y_;
  }
  double Odometry::GetVrz()
  {
    return angular_;
  }
//}