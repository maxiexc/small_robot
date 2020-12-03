/**
 * @defgroup   JOY_CONTROL_LLC_NODE joy control llc node
 *
 * @brief      This file implements joy control llc node.
 *
 * @author     Maxie
 * @date       2020.11.25
 * 
 * @todo       Add parameter.
 */

/*Standard include*/
//#include <unistd.h>

/*Board specific include*/

extern "C"
{  
#include "rc_usefulincludes.h"
#include "roboticscape.h"
}

/*ROS specific include*/
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>   //Add on 2020.11.28
#include <nav_msgs/Odometry.h>          //Add on 2020.11.28

/*Application include*/
extern "C"
{ 
#include "motor_controller.h"
#include "pd_controller.h"
#include "motor_config.h" 
}
#include "odometry.h"                   //Add on 2020.11.28

/*Define config values*/
#define SAMPLE_RATE_HZ          50
#define ROS_MSG_PUBLISH_RATE    25
#define ROS_MAIN_LOOP_RATE      50
#define MOTOR_CONTROLLER_RATE   100
#define JCLLC_CMD_TMO_THOLD     1.0

/*Application specific include*/
//using namespace std;

/*Struct*/
typedef struct
{
  volatile bool motor_ena;  /*Motor enable flag*/
  float speed_linear_cmd;
  float speed_angular_cmd;
  int32_t speed1_cmd_pps;
  int32_t speed2_cmd_pps;
  double tstamp_twist_cmd;
  pthread_mutex_t lock_motor_ena;
} LLC_HANDLE_T;

typedef struct
{
  volatile uint32_t counter;
  volatile bool is_timeout;
  volatile bool is_set;
} LLC_VAR_HANDLE_T;


typedef struct
{
  int     motor_pulse_per_meter;
  int     motor_speed_max_abs;
  int     motor_speed_min_abs;
  double  track_width_meter;
  double  motor_dduty_max001;
  double  motor_speed_controller_kp;
  double  motor_speed_controller_kd;
} LLC_PARAMETER_T;

/*Global variables*/
LLC_HANDLE_T llc = {0};
LLC_PARAMETER_T param = {0};
/*Mutexes*/
//pthread_mutex_t imu_mutex = PTHREAD_MUTEX_INITIALIZER; /**< Mutex for imu_msg*/

/*Local variables*/
sensor_msgs::Imu imu_msg;
rc_imu_data_t data = {0};
LLC_VAR_HANDLE_T imu_var = {0};
std::string frame_id = "imu_base";    /**< Frame ID of IMU*/
MOTC_HANDLE_T motc1;                  /**< Motor controller handle of motor 1*/
MOTC_HANDLE_T motc2;                  /**< Motor controller handle of motor 2*/
/*Declare publisher*/
ros::Publisher imu_pub;
ros::Publisher odom_pub;

/*Function declaration*/
/*Static function*/
static int BBBL_InitPeripheral(void);
static int InitMotorController(void);
static void PrintParam(void);
static void SetParam(void);

/*Non static function*/
void IMU_InterruptCB(void);
void ros_compatible_shutdown_signal_handler(int signo);
void TwistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_twist);
uint8_t CheckMsgTMO(const double &tstamp, const double tmo_threshold);
uint8_t SetMotorCmd(void);
void StopBothMotor(void);
/*Threads*/
void* tPublishStatus(void* ptr);
void* tMotorController(void* arg);

/*main*/
int main(int argc, char *argv[])
{
  /*Initialize structure*/
  llc.motor_ena = false;
  llc.lock_motor_ena = PTHREAD_MUTEX_INITIALIZER;
  
  /*Priveate variables*/
  bool priv_motor_ena = false;

  /*Init ROS*/
  ros::init(argc, argv, "small_robot_llc");

  /*Create and start the noede*/
  ros::NodeHandle nh;

  /*Process parameter*/
  SetParam();
  PrintParam();
  /*Create publisher*/
  /*Create publisher imu to publish sensor_msgs/imu */
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  /*Create subscriber twist_sub to subcribe to geometry_msgs/Twist*/
  ros::Subscriber sub_twist = nh.subscribe("cmd_vel", 100, TwistCallback);
  ros::Rate loop_rate(ROS_MAIN_LOOP_RATE);

  /*Initialize BBBL*/
  if(rc_initialize()){
    ROS_ERROR ("ERROR: failed to initialize cape.\n");
    return -1;
  }

  /*Initializa BBBL peripheral*/
  if(BBBL_InitPeripheral())
  {
    ROS_ERROR("ERROR: BBBL peripheral initialization failed");
    return -1;
  }

  /*Initialize motor controller*/
  if(InitMotorController())
  {
    ROS_ERROR("ERROR: Motor controller initialization failed");
    return -1;
  }
  
  /*Set CB function*/
  signal(SIGINT,  ros_compatible_shutdown_signal_handler);  
  signal(SIGTERM, ros_compatible_shutdown_signal_handler);  


  /*Init mutex*/
  /*mutex init through macro*/
  

  /*Set and start threads*/
  /*Set and start a publish thread to publish message*/
  pthread_t  printf_thread;
  pthread_create(&printf_thread, NULL, tPublishStatus, (void*) NULL);
  
  /*Set and start a command thread to command motor*/
  pthread_t mot_ctrl_thread;
  pthread_create(&mot_ctrl_thread, NULL, tMotorController, (void*)NULL);

  /*Set state to running and start*/
  rc_set_state(RUNNING);

  while(ros::ok())
  {
    /*Reveice*/
    ros::spinOnce();

    pthread_mutex_lock(&(llc.lock_motor_ena));
    /*Critical section*/
    priv_motor_ena = llc.motor_ena;
    pthread_mutex_unlock(&(llc.lock_motor_ena));

    /*Process*/
    if(CheckMsgTMO(llc.tstamp_twist_cmd, JCLLC_CMD_TMO_THOLD))
    {
      /*Stop motor*/
      if(priv_motor_ena)
      {
        /*Switch motor_ena to 0*/
        priv_motor_ena = false;
      }
      llc.speed1_cmd_pps = 0;
      llc.speed2_cmd_pps = 0;
    }

    /*Command*/
    if(SetMotorCmd())
    {
      ROS_ERROR ("ERROR: failed to set command of motor controller.\n");
      priv_motor_ena = false;
    }

    /*Update*/
    pthread_mutex_lock(&(llc.lock_motor_ena));
    /*Critical section*/
    llc.motor_ena = priv_motor_ena;
    pthread_mutex_unlock(&(llc.lock_motor_ena));

    /*Sleep*/
    loop_rate.sleep();
  }

  /*Node exit procedure*/
  ROS_INFO("Node prepare to exit\n");
  priv_motor_ena = false;
  pthread_mutex_lock(&(llc.lock_motor_ena));
  llc.motor_ena = priv_motor_ena;
  pthread_mutex_unlock(&(llc.lock_motor_ena));
  //StopBothMotor();
  rc_set_state(EXITING);
  ros::shutdown();

  // exit cleanly
  rc_power_off_imu();
  rc_cleanup();

  ROS_INFO("Node exit\n");
  return 0;
}

/**
 * @brief      This function update imu data to sensor_msgs::Imu.
 *             
 * @details    This function update [x, y, z, w] of quat and set imu_var.is_set to true.
 *             This function will also update a counter after it updated imu data.
 *             
 * @note       This funciton access variable used by other thread directly.
 * 
 */
void IMU_InterruptCB(void)
{
  /*Check for exit condition*/
  if(rc_get_state()==EXITING){
    ROS_INFO("IMU callback exit detected\n");
    return;
  }
  /*Copy IMU data to ROS message*/
  imu_msg.orientation.x = data.dmp_quat[QUAT_X];
  imu_msg.orientation.y = data.dmp_quat[QUAT_Y];
  imu_msg.orientation.z = data.dmp_quat[QUAT_Z];
  imu_msg.orientation.w = data.dmp_quat[QUAT_W];
  imu_msg.header.frame_id = frame_id;
  imu_var.is_set = true;
  imu_var.counter++;
}

/**
 * @brief      This function process the subscribe topic "/twist".
 *             This function does convert linear.x and angular.z into s_mot1 and s_mot2.
 *
 * @param[in]  cmd_vel_twist  The command velocity twist
 */
void TwistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_twist)
{
  int32_t speed_mot1_pps = 0.0f;
  int32_t speed_mot2_pps = 0.0f;
  double speed_linear_cmd = 0.0;
  double speed_angular_cmd = 0.0;
  /*Receive messages*/
  pthread_mutex_lock(&(llc.lock_motor_ena));
  /*Critical section*/
  llc.motor_ena = true;
  pthread_mutex_unlock(&(llc.lock_motor_ena));
  //printf("twist received\n");
  speed_linear_cmd   = cmd_vel_twist->linear.x;
  speed_angular_cmd = cmd_vel_twist->angular.z;

  /*Deadzone detection*/
  if((speed_linear_cmd < 0.07) && (speed_linear_cmd > -0.07))
  {
    speed_linear_cmd = 0.0;
  }

  /*Deadzone detection*/
  if((speed_angular_cmd < 0.1) && (speed_angular_cmd > -0.1))
  {
    speed_angular_cmd = 0.0;
  } 

  llc.speed_linear_cmd   = speed_linear_cmd;
  llc.speed_angular_cmd = speed_angular_cmd;
  llc.tstamp_twist_cmd = ros::Time::now().toSec();

  /*Conver twist message into pps of each wheel*/
  speed_mot1_pps = (int32_t)((llc.speed_linear_cmd * (float)(param.motor_pulse_per_meter)) + (float)((double)llc.speed_angular_cmd * 0.5 * (param.motor_pulse_per_meter) * (param.track_width_meter)));
  speed_mot2_pps = (int32_t)((llc.speed_linear_cmd * (float)(param.motor_pulse_per_meter)) - (float)((double)llc.speed_angular_cmd * 0.5 * (param.motor_pulse_per_meter) * (param.track_width_meter)));
  
  /*Deadzone detection*/
  if(abs(speed_mot1_pps) < 100)
  {
    speed_mot1_pps = 0;
  }

  /*Deadzone detection*/
  if(abs(speed_mot2_pps) < 100)
  {
    speed_mot2_pps = 0;
  }

  llc.speed1_cmd_pps = speed_mot1_pps;
  llc.speed2_cmd_pps = speed_mot2_pps;
  if(abs(llc.speed1_cmd_pps) < param.motor_speed_min_abs)
  {
    llc.speed1_cmd_pps = 0;
  }

  if(abs(llc.speed2_cmd_pps) < param.motor_speed_min_abs)
  {
    llc.speed2_cmd_pps = 0;
  }
}

/**
 * @brief      This funciton check if timeout condition is true
 *
 * @param[in]  tstamp            The time stamp
 * @param[in]  tmo_threshold     The timeout  threshold
 *
 * @return     0:     Timeout condition is false
 *             other: Timeout condition is true
 */
uint8_t CheckMsgTMO(const double &tstamp, const double tmo_threshold)
{
  double time_now = ros::Time::now().toSec();      
  double t_pass = time_now - tstamp;

  //printf("Time pass: %6.6lf. Threshold:  %6.6lf\n",t_pass, tmo_threshold);

  if(t_pass > tmo_threshold)
  {
    /*timeout is true*/
    return 1U;
  }

  return 0U;
}

/**
 * @brief      Ths function set speed command value of motor.
 *
 * @return     { description_of_the_return_value }
 */
uint8_t SetMotorCmd(void)
{
  uint8_t result = 0U;
  if(MOTC_SetCmd(&motc1, llc.speed1_cmd_pps))
  {
    /*Faild to run MOTC_SetCmd*/
    result += 1U;
  }
  if(MOTC_SetCmd(&motc2, llc.speed2_cmd_pps))
  {
    /*Faild to run MOTC_SetCmd*/
    result += 1U;
  }
  return result;
}


void StopBothMotor(void)
{
  ROS_INFO("Stop motor,\n");
  rc_set_motor_brake(motc1.cfg.motor_id);
  rc_set_motor_brake(motc2.cfg.motor_id);
  rc_usleep(1000000);
  rc_set_motor_free_spin(motc1.cfg.motor_id);
  rc_set_motor_free_spin(motc2.cfg.motor_id);
}

/**
 * @brief      This is a thread that publish status related message
 * 
 * @note       This thread runs at 50 Hz * 
 *
 * @param      arg  Argument
 *
 * @return     { description_of_the_return_value }
 */
void* tPublishStatus(void* arg)
{
  /*Declare tf broadcaster*/
  tf::TransformBroadcaster odom_broadcaster;
  rc_state_t last_rc_state, new_rc_state; //keep track of last state
  sensor_msgs::Imu local_imu_msg;         //imu message
  geometry_msgs::TransformStamped odom_trans; //Trasform of odom
  nav_msgs::Odometry odom;                //Odometry message
  ros::Time time_now = ros::Time::now(); 
  uint32_t priv_counter = 0U;
  
  Odometry odometry(TRACK_WIDTH_M, MOTOR_PULSE_PER_METER);
  odometry.Init(time_now);


  new_rc_state = rc_get_state();

  while(rc_get_state()!=EXITING)
  {
    last_rc_state = new_rc_state; 
    new_rc_state = rc_get_state();
    time_now = ros::Time::now();

    //Publish imu data
    if(true == imu_var.is_set)
    { 
      priv_counter = imu_var.counter;
      local_imu_msg= imu_msg;

      if(priv_counter != imu_var.counter)
      {
        /*message has been modified during copy, copy again*/
        local_imu_msg= imu_msg;
      }

      imu_pub.publish(local_imu_msg);
      imu_var.is_set = false;
      imu_var.is_timeout = false;
    }
    else{
      /*No IMU update detected*/
      if(false == imu_var.is_timeout)
      {
        ROS_WARN("IMU update timeout");
        imu_var.is_timeout = true;
      }
    }

    //Set and publish tf message
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry.GetHeading());
    odom_trans.header.stamp = time_now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odometry.GetX();;
    odom_trans.transform.translation.y = odometry.GetY();;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //Set and publish odom message
    odometry.Update((double)(rc_get_encoder_pos(motc2.cfg.encoder_id)), (double)rc_get_encoder_pos(motc1.cfg.encoder_id) * (-1.0), time_now);
    odom.header.stamp = time_now;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = odometry.GetX();
    odom.pose.pose.position.y = odometry.GetY();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odometry.GetVx();
    odom.twist.twist.linear.y = odometry.GetVy();
    odom.twist.twist.angular.z = odometry.GetVrz();
    odom_pub.publish(odom);

    //Sleep
    rc_usleep(1000000 / SAMPLE_RATE_HZ);
  }
  
  pthread_exit(NULL);
}

void* tMotorController(void *arg)
{
  /*Cast arg into proper type*/
  (void*) arg;

  /*Setup private variables*/
  const uint32_t time_thd_start = (uint32_t)(rc_nanos_since_boot()/1000000ULL);
  uint32_t time_thd_now = time_thd_start;
  bool priv_motor_ena = false;
  bool priv_motor_ena_last = false;
  uint16_t priv_motor_stop_count = 0U;

  /*Init thread*/

  /*Thread start looping*/
  while(rc_get_state() != EXITING)
  {
    //pthread_mutex_lock(&g_ctrl.lock);
    /*Critical section*/
    /*Update motor command*/
    priv_motor_ena_last = priv_motor_ena;

    pthread_mutex_lock(&(llc.lock_motor_ena));
    /*Critical section*/
    priv_motor_ena = llc.motor_ena;
    pthread_mutex_unlock(&(llc.lock_motor_ena));

    //std::cout<<"priv_motor_ena is: "<<priv_motor_ena<<std::endl;
    //printf("priv_motor_ena is %d\n", priv_motor_ena);
    //pthread_mutex_unlock(&g_ctrl.lock);

    if(priv_motor_ena)
    {
      if(!priv_motor_ena_last)
      {
        //ROS_INFO("Start motor command detected");
      }
      time_thd_now = (uint32_t)(rc_nanos_since_boot()/1000000ULL) - time_thd_start;
      //printf("MOT1 cmd:%d\n", motc1.cmd.speed);
      //printf("MOT2 cmd:%d\n", motc2.cmd.speed);
      MOTC_SetEncoder(&motc1, rc_get_encoder_pos(motc1.cfg.encoder_id), time_thd_now);
      MOTC_SetEncoder(&motc2, rc_get_encoder_pos(motc2.cfg.encoder_id), time_thd_now);

      if(!motc1.cmd.speed)
      {
        //printf("Free spin mot 1\n");
        rc_set_motor_free_spin(motc1.cfg.motor_id);
      }
      else
      {
        if(MOTC_Run(&motc1))
        {
          printf("Error occurred during MOTC_RunPDC\n");
        }
        //printf("MOT%d duty:%f\n",motc1.cfg.motor_id, motc1.cmd_out);
        rc_set_motor(motc1.cfg.motor_id, motc1.cmd_out);      
      }

      if(!motc2.cmd.speed)
      {
        //printf("Free spin mot 2\n");
        rc_set_motor_free_spin(motc2.cfg.motor_id);
      }
      else
      {
        if(MOTC_Run(&motc2))
        {
          printf("Error occurred during MOTC_RunPDC\n");
        }
        //printf("MOT%d duty:%f\n",motc2.cfg.motor_id, motc2.cmd_out);
        rc_set_motor(motc2.cfg.motor_id, motc2.cmd_out);
      }
      priv_motor_stop_count = 0;
    }
    else
    {
      if(priv_motor_ena_last)
      {
        //ROS_INFO("Stop motor command detected");
      }
      /*Stop both motor*/
      priv_motor_stop_count += 1U;
      if(priv_motor_stop_count >= 100U)
      {
        rc_set_motor_free_spin(motc1.cfg.motor_id);
        rc_set_motor_free_spin(motc2.cfg.motor_id);
      }
      else
      {
        rc_set_motor_brake(motc1.cfg.motor_id);
        rc_set_motor_brake(motc2.cfg.motor_id);
      }
    }

    /*Sleep*/
    rc_usleep(1000000 / MOTOR_CONTROLLER_RATE);
  }

  pthread_exit(NULL);
}

/*******************************************************************************
 * shutdown_signal_handler(int signo)
 *
 * catch Ctrl-C signal and change system state to EXITING
 * all threads should watch for get_state()==EXITING and shut down cleanly
 *******************************************************************************/
void ros_compatible_shutdown_signal_handler(int signo)
{
  if (signo == SIGINT)
  {
    rc_set_state(EXITING);
    ROS_INFO("\nReceived SIGINT Ctrl-C.");
    ros::shutdown();
  }
  else if (signo == SIGTERM)
  {
    rc_set_state(EXITING);
    ROS_INFO("Received SIGTERM.");
    ros::shutdown();
  }
}

/**
 * @brief      This function help initilize BBBL related peripheral
 *
 * @return     Status of initialization.
 *             0: Initialize complete
 *            -1: Error occurred  
 */
static int BBBL_InitPeripheral(void)
{
  /*Setup IMU*/
  rc_imu_config_t imu_config = rc_default_imu_config();
  imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;

  /*Initilalize imu DMP*/
  if(rc_initialize_imu_dmp(&data, imu_config)){
    ROS_WARN("WARN: rc_initialize_imu_dmp() falied\n");
    return -1;
  }

  rc_set_imu_interrupt_func(&IMU_InterruptCB);
  rc_enable_motors();
  return 0;
}

/**
 * @brief      This funciton help initialize motor controller and speed controller
 *
 * @return     Status of initialization.
 *             0: Initialize complete
 *            -1: Error occurred  
 */
static int InitMotorController(void)
{
  if(MOTC_Init(&motc1, MOTC_MODE_ENUM_SPEED, 1, 1, 1, MOTOR_PULSE_PER_REV, (float)param.motor_dduty_max001, MOTOR_DUTY_MAX_ABS, MOTOR_DUTY_MIN_ABS))
  {
    ROS_ERROR("Error occurred during MOTC_Init on motor 1\n");
    return -1;
  }

  if(MOTC_Init(&motc2, MOTC_MODE_ENUM_SPEED, 0, 2, 2, MOTOR_PULSE_PER_REV, (float)param.motor_dduty_max001, MOTOR_DUTY_MAX_ABS, MOTOR_DUTY_MIN_ABS))
  {
    ROS_ERROR("Error occurred during MOTC_Init on motor 2\n");
    return -1;
  }

  if(MOTC_SetSpeedController(&motc1, 0U, (float)param.motor_speed_controller_kp, (float)param.motor_speed_controller_kd))
  {
    ROS_ERROR("Error occurred during MOTC_SetSpeedPDC\n");
    return -1;
  }

  if(MOTC_SetSpeedController(&motc2, 0U, (float)param.motor_speed_controller_kp, (float)param.motor_speed_controller_kd))
  {
    ROS_ERROR("Error occurred during MOTC_SetSpeedPDC\n");
    return -1;
  }

  return 0;
}


static void PrintParam(void)
{
  ROS_INFO("motor_pulse_per_meter is: %d", param.motor_pulse_per_meter);
  ROS_INFO("motor_speed_max_abs is: %d", param.motor_speed_max_abs);
  ROS_INFO("motor_speed_min_abs is: %d", param.motor_speed_min_abs);
  ROS_INFO("track_width_meter is: %lf", param.track_width_meter);
  ROS_INFO("motor_dduty_max001 is: %lf", param.motor_dduty_max001);
  ROS_INFO("motor_speed_controller_kp is: %lf", param.motor_speed_controller_kp);
  ROS_INFO("motor_speed_controller_kd is: %lf", param.motor_speed_controller_kd);
}

static void SetParam(void)
{
  ros::NodeHandle priv_nh("~");
  priv_nh.param<int>("motor_pulse_per_meter", param.motor_pulse_per_meter, MOTOR_PULSE_PER_METER);
  priv_nh.param<int>("motor_speed_max_abs", param.motor_speed_max_abs, MOTOR_SPEED_MAX_ABS);
  priv_nh.param<int>("motor_speed_min_abs", param.motor_speed_min_abs, MOTOR_SPEED_MIN_ABS);
  priv_nh.param<double>("track_width_meter", param.track_width_meter, TRACK_WIDTH_M);
  priv_nh.param<double>("motor_dduty_max001", param.motor_dduty_max001, MOTOR_DDUTY_MAX001);
  priv_nh.param<double>("motor_speed_controller_kp", param.motor_speed_controller_kp, MOTOR_SPEED_CONTROLLER_KP);
  priv_nh.param<double>("motor_speed_controller_kd", param.motor_speed_controller_kd, MOTOR_SPEED_CONTROLLER_KD);
}
