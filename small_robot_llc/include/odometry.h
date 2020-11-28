#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct
{
  double x;
  double y;
  double heading;
} ODOM_POSE_T;

namespace small_robot_llc
{
  class Odometry
  {
  public:
    /*Constructor*/
    Odometry();

    void Init();
    void Update();
    
    void SetX();
    void SeyY();
    void SetHeading();

    double GetX();
    double GetY();
    double GetHeading();

  private:
    //Parameters
    double track_width;

    // Position
    ODOM_POSE_T pose_now;
    ODOM_POSE_T pose_last;
    
    // Velocity
    double linear;
    double angular;

    // Time
    ros::time time_now;
    ros::time time_last;
  }
}
#endif