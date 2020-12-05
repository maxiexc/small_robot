/**
 * @defgroup   TF_BROADCASTER tf broadcaster
 *
 * @brief      This file implements a tf broadcaster.
 *             basel_link is the location of IMU.
 *             base_laser is the center of Lidar. Cordinate of base_laser is [-0.03, +0.0, +0.065]
 *
 * @note       This code is modied from tutorial.
 *             The link of tutorial is "http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF"
 *             
 * @author     Maxie
 * @date       2020.11.28
 * 
 * @version    0.01
 */


/*ros related include*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    /*x = -30mm y = 0mm z = +72.8mm*/
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.03, 0.0, 0.0728)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
}