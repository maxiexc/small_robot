#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct
{
  double x;
  double y;
  double heading;
} OdomPoseT;

namespace small_robot_llc
{
  class Odometry
  {
  public:
    //Constructor
    Odometry();

    void Init();
    bool Update();
    
    void SetX();
    void SeyY();
    void SetHeading();

    double GetX();
    double GetY();
    double GetHeading();

  private:
    //Parameters
    double track_width_;
    double enc_count_per_meter_;

    //Position
    OdomPoseT pose_now_;
    OdomPoseT pose_last_;

    double enc_l_last;
    double enc_r_last;
    //Velocity
    double linear_;
    double angular_;

    //Time
    ros::time time_now_;
    ros::time time_last_;
  }
}
#endif