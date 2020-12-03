/**
 * @defgroup   ODOMETRY odometry
 *
 * @brief      This file implements odometry.
 *
 * @author     Maxie
 * @date       2020.11.29
 * @version    0.0.1
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

/*ROS include*/
#include "ros/ros.h"

typedef struct
{
  double x;
  double y;
  double heading;
} OdomPoseT;

//namespace small_robot_llc
//{
  class Odometry
  {
  public:
    //Constructor
    Odometry(const double track_width = 0.0, const double enc_count_per_meter = 0.0);

    void Init(const ros::Time &time);
    bool Update(const double enc_l, const double enc_r, const ros::Time &time_now);
    
    void SetX(double x);
    void SetY(double y);
    void SetHeading(double heading);

    double GetX();
    double GetY();
    double GetHeading();

    double GetVx();
    double GetVy();
    double GetVrz();

  private:
    //Parameters
    double track_width_;
    double enc_count_per_meter_;

    //Position
    OdomPoseT pose_now_;
    OdomPoseT pose_last_;

    double enc_l_last_;
    double enc_r_last_;
    //Velocity
    double linear_;
    double linear_x_;
    double linear_y_;
    double angular_;

    //Time
    ros::Time time_now_;
    ros::Time time_last_;

    //Methods
    void IntegrateRungeKutta2(double mov_linear, double mov_angular);
    void IntegrateExact(double mov_linear, double mov_angular);

  };
//}
#endif