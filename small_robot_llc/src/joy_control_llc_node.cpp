/**
 * @defgroup   JOY_CONTROL_LLC_NODE joy control llc node
 *
 * @brief      This file implements joy control llc node.
 *
 * @author     Maxie
 * @date       2020.11.08
 */

/*Standard include*/
#include <unistd.h>

/*Board specific include*/
extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{  
#include "roboticscape.h"
}

/*ROS specific include*/
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

/*Application include*/
extern "C"
{  
#include "motor_controller.h"
#include "pd_controller.h"
}
#include "motor_config.h"

/*Define config values*/
#define SAMPLE_RATE_HZ          50
#define ROS_MSG_PUBLISH_RATE    25
#define ROS_MAIN_LOOP_RATE      50
#define MOTOR_CONTROLLER_RATE   100
#define JCLLC_CMD_TMO_THOLD_F32 1.0f

/*Application specific include*/

/*Struct*/
typedef struct
{
  uint32_t motor_ena;
  float speed_linear_cmd;
  float speed_angular_cmd;
  int32_t speed1_cmd_pps;
  int32_t speed2_cmd_pps;
  float tstamp_twist_cmd;
  pthread_mutex_t lock;

} LLC_HANDLE_T;

/*Global variables*/
LLC_HANDLE_T llc = {0};

/*Mutexes*/
pthread_mutex_t imu_mutex = PTHREAD_MUTEX_INITIALIZER; /**< Mutex for imu_msg*/

/*Local variables*/
sensor_msgs::Imu imu_msg;
rc_imu_data_t data = {0}; 
volatile bool imu_is_set = false;
std::string frame_id = "imu_frame";   /**< Frame ID of IMU*/
MOTC_HANDLE_T motc1;                  /**< Motor controller handle of motor 1*/
MOTC_HANDLE_T motc2;                  /**< Motor controller handle of motor 2*/
/*Declear publisher*/
ros::Publisher imu_pub;

/*Functions declaration*/
void IMU_InterruptCB(void);
void ros_compatible_shutdown_signal_handler(int signo);
void TwistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_twist);
uint8_t CheckMsgTMO(const float tstamp, const float tmo_threshold);
uint8_t SetMotorCmd(void);
void StopBothMotor(void);

/*Threads*/
void* tPublishStatus(void* ptr);
void* tMotorController(void* arg);

/*main*/
int main(int argc, char *argv[])
{

  /*Init ROS*/
  ros::init(argc, argv, "bbbl_imu");

  /*Create and start the noede*/
  ros::NodeHandle nh;

  /*Create publisher*/
  /*No publisher at the moment*/

  /*Create subscriber twist_sub to subcribe to geometry_msgs/Twist*/
  ros::Subscriber sub_twist = nh.subscribe("cmd", 100, TwistCallback);

  ros::Rate loop_rate(ROS_MAIN_LOOP_RATE);

  /*Initialize cape lib*/
  if(rc_initialize()){
    ROS_ERROR ("ERROR: failed to initialize cape.\n");
    return -1;
  }

  /*Initialize motor controller*/
  if(MOTC_Init(&motc1, MOTC_MODE_ENUM_SPEED, 1, 1, 1, 2800, MOTOR_DDUTY_MAX001, MOTOR_DUTY_MAX_ABS, MOTOR_DUTY_MIN_ABS))
  {
    ROS_ERROR("Error occurred during MOTC_Init on motor 1\n");
  }

  if(MOTC_Init(&motc2, MOTC_MODE_ENUM_SPEED, 0, 2, 2, 2800, MOTOR_DDUTY_MAX001, MOTOR_DUTY_MAX_ABS, MOTOR_DUTY_MIN_ABS))
  {
    ROS_ERROR("Error occurred during MOTC_Init on motor 2\n");
  }

  if(MOTC_SetSpeedController(&motc1, 0U, 0.02f, 0))
  {
    ROS_ERROR("Error occurred during MOTC_SetSpeedPDC\n");
  }

  if(MOTC_SetSpeedController(&motc2, 0U, 0.02f, 0))
  {
    ROS_ERROR("Error occurred during MOTC_SetSpeedPDC\n");
  }

  /*Init mutex*/
  pthread_mutex_init(&llc.lock, NULL);

  /*Setup IMU*/
  rc_imu_config_t imu_config = rc_default_imu_config();
  imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;

  /*Initilalize imu DMP*/
  if(rc_initialize_imu_dmp(&data, imu_config)){
    ROS_WARN("WARN: rc_initialize_imu_dmp() falied\n");
    return -1;
  }

  /*Set CB function*/
  signal(SIGINT,  ros_compatible_shutdown_signal_handler);  
  signal(SIGTERM, ros_compatible_shutdown_signal_handler);  
  rc_set_imu_interrupt_func(&IMU_InterruptCB);

  /*Set and start threads*/
  /*Set and start a publish thread to publish message*/
  //pthread_t  printf_thread;
  //pthread_create(&printf_thread, NULL, tPublishStatus, (void*) NULL);
  
  /*Set and start a command thread to command motor*/
  pthread_t mot_ctrl_thread;
  pthread_create(&mot_ctrl_thread, NULL, tMotorController, (void*)NULL);

  /*Set state to running and start motor*/
  rc_set_state(RUNNING);
  rc_enable_motors();


  while(ros::ok())
  {
    /*Reveice*/
    ros::spinOnce();
    /*Process*/
    if(CheckMsgTMO(llc.tstamp_twist_cmd, JCLLC_CMD_TMO_THOLD_F32))
    {
      /*Send timeout message and stop motor*/
      //ROS_INFO("Twist command timeout");
      llc.motor_ena = 0;
      llc.speed1_cmd_pps = 0;
      llc.speed2_cmd_pps = 0;
    }
    /*Command*/
    if(SetMotorCmd())
    {
      ROS_ERROR ("ERROR: failed to set command of motor controller.\n");
      llc.motor_ena = 0;
    }
    /*Sleep*/
    loop_rate.sleep();
  }

  ROS_INFO("Node prepare to exit\n");
  llc.motor_ena = 0;
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
 * @brief   This function update imu data to sensor_msgs::Imu
 * @details This function update [x, y, z, w] of quat and set imu_is_set to true
 */
void IMU_InterruptCB(void)
{
  /*Check for exit condition*/
  if(rc_get_state()==EXITING){
    ROS_INFO("IMU callback exit detected\n");
    return;
  }

  /*Copy IMU data to ROS message*/
  pthread_mutex_lock(&imu_mutex);
  imu_msg.orientation.x = data.dmp_quat[QUAT_X];
  imu_msg.orientation.y = data.dmp_quat[QUAT_Y];
  imu_msg.orientation.z = data.dmp_quat[QUAT_Z];
  imu_msg.orientation.w = data.dmp_quat[QUAT_W];
  imu_msg.header.frame_id = frame_id;
  imu_is_set = true;
  pthread_mutex_unlock(&imu_mutex);
}

/**
 * @brief      This function process the subscribe topic "/twist".
 *             This function does convert linear.x and angular.z into s_mot1 and s_mot2.
 *
 * @param[in]  cmd_vel_twist  The command velocity twist
 */
void TwistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_twist)
{
  float speed_mot1_pps = 0.0f;
  float speed_mot2_pps = 0.0f;
  /*Receive messages*/
  llc.motor_ena = 1;
  llc.speed_linear_cmd   = cmd_vel_twist->linear.x;
  llc.speed_angular_cmd = cmd_vel_twist->angular.z;
  llc.tstamp_twist_cmd = ros::Time::now().toSec();

  /*Conver twist message into pps of each wheel*/
  speed_mot1_pps = (llc.speed_linear_cmd * (float)MOTOR_PULSE_PER_METER) + (llc.speed_angular_cmd * MOTOR_PULSE_PER_METER * TRACK_WIDTH_M);
  speed_mot2_pps = (llc.speed_linear_cmd * (float)MOTOR_PULSE_PER_METER) - (llc.speed_angular_cmd * MOTOR_PULSE_PER_METER * TRACK_WIDTH_M);
  llc.speed1_cmd_pps = (int32_t)speed_mot1_pps;
  llc.speed2_cmd_pps = (int32_t)speed_mot2_pps;
  if(abs(llc.speed1_cmd_pps) < MOTOR_SPEED_MIN_ABS)
  {
    llc.speed1_cmd_pps = 0;
  }

  if(abs(llc.speed2_cmd_pps) < MOTOR_SPEED_MIN_ABS)
  {
    llc.speed2_cmd_pps = 0;
  }
}

/**
 * @brief      This funciton check if timeout condition is true
 *
 * @param[in]  tstamp            The time stamp
 * @param[in]  tmo_threshold     The time out  threshold
 *
 * @return     0:     Timeout condition is false
 *             other: Timeout condition is true
 */
uint8_t CheckMsgTMO(const float tstamp, const float tmo_threshold)
{
  float time_now = ros::Time::now().toSec();      
  float t_pass = time_now - tstamp;

  if(t_pass > tmo_threshold)
  {
    /*Time out is true*/
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
  //pthread_mutex_lock(&g_ctrl.lock);
  ///*Critical section*/
  //g_ctrl.motor_ena = 0;
  //pthread_mutex_unlock(&g_ctrl.lock);
  //rc_usleep(500000);
}

/*Not used at the moment*/
void* tPublishStatus(void* ptr)
{
  rc_state_t last_rc_state, new_rc_state; // keep track of last state
  sensor_msgs::Imu local_imu_msg = imu_msg;


  new_rc_state = rc_get_state();
  
  while(rc_get_state()!=EXITING)
  {
    last_rc_state = new_rc_state; 
    new_rc_state = rc_get_state();

    pthread_mutex_lock(&imu_mutex);
    
    if(imu_is_set == true)
    {
      imu_pub.publish(local_imu_msg);
      imu_is_set = false;
    }
    else{
      /*No IMU update detected*/
      ROS_INFO("IMU update timeout");
    }
    pthread_mutex_unlock(&imu_mutex);

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
  int32_t priv_motor_ena = 0U;
  uint16_t priv_motor_stop_count = 0U;

  /*Init thread*/

  /*Thread start looping*/
  while(rc_get_state() != EXITING)
  {
    //pthread_mutex_lock(&g_ctrl.lock);
    /*Critical section*/
    /*Update motor command*/
    priv_motor_ena = llc.motor_ena;
    //pthread_mutex_unlock(&g_ctrl.lock);

    if(1 == priv_motor_ena)
    {
      time_thd_now = (uint32_t)(rc_nanos_since_boot()/1000000ULL) - time_thd_start;
      printf("MOT1 cmd:%d\n", motc1.cmd.speed);
      printf("MOT2 cmd:%d\n", motc2.cmd.speed);
      MOTC_SetEncoder(&motc1, rc_get_encoder_pos(motc1.cfg.encoder_id), time_thd_now);
      MOTC_SetEncoder(&motc2, rc_get_encoder_pos(motc2.cfg.encoder_id), time_thd_now);

      if(MOTC_Run(&motc1))
      {
        printf("Error occurred during MOTC_RunPDC\n");
      }

      if(MOTC_Run(&motc2))
      {
        printf("Error occurred during MOTC_RunPDC\n");
      }
      printf("MOT%d duty:%f\n",motc1.cfg.motor_id, motc1.cmd_out);
      printf("MOT%d duty:%f\n",motc2.cfg.motor_id, motc2.cmd_out);
      rc_set_motor(motc1.cfg.motor_id, motc1.cmd_out);
      rc_set_motor(motc2.cfg.motor_id, motc2.cmd_out);
      priv_motor_stop_count = 0;
    }
    else
    {
      /*Stop both motor*/
      priv_motor_stop_count += 1U;
      if(priv_motor_stop_count >= 200U)
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
