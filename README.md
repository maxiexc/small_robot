# Small Robot

## Package: small_robot_llc
This is a ROS package contain low level control algrithem of small robot.
This package is designed for Beaglebone blue

### launch files
- small_robot: This launch file will start "sr_llc_node", X2L and a static tf broadcaster that broadcast tf between "base_link" and "laser_frame".
- X2L - This is a launch file to start YDLidar-X2L and it's corresponding ros node.

### Subsrcibtion topics
cmd_vel: geometry_msgs/Twist

### Publish topics
imu: sensor_msgs/Imu
odom: nav_msgs/Odometry

### Actions
- Control motor 1 and 2 acoording to command from cmd_vel.
- Read from IMU and publish.
- Calculate odometry and publish.

### Parameters
#### motor_pulse_per_meter (int, default: 21738)
  Motor encoder pulse counts per 1 meter.

#### motor_speed_max_abs (int, default: 8000)
  Motor maximum aboslute speed in pulse count.  

#### motor_speed_min_abs (int, default: 100)
  Motor minimum aboslute speed in pulse count.

#### track_width_meter (double, default: 0.125)
  Track width in meter.

#### motor_dduty_max001 (double, default: 20.0)
  Maximum single iteration value of motor duty cycle in unit of 0.01.

#### motor_speed_controller_kp (double, default: 0.015)
  Kp of motor speed controller.

#### motor_speed_controller_kd (double, default: 0.0)
  Kd of motor speed controller.

## Package: robot_setup_tf
This is a ROS package contain static tf broadcaster required by navigation.
This package is currently not in used since static tf broadcaster is currently written in `*.launch` files.

## Package: small_robot_2dnav
This package is created according to [tutorial](http://wiki.ros.org/navigation/Tutorials/RobotSetup). Several parameters are modified to fit the performance of Beaglebone blue.

### Launch files
**It is required to modify `args` within map_server to path of desired map**
- move_base: This launch file open rerquired node for navigation. Static map (map server + amcl) is used.
- small_robot_configuration: This launch file launch "small_robot_llc/small_robot.launch".

## Package: small_robot_2dnav_remote
This package is similar "to small_robot_2dnav" but modified to run on laptop rather than Beaglebone blue. Several parameters are modified to fit the performence of laptop.

### Launch files
**It is required to modify `args` within map_server to path of desired map**
- ground_station: This launch files open rviz with configuration for navigation.
- move_base_remote: This launch file open rerquired node for navigation. Static map (map server + amcl) is used. running "small_robot_llc/small_robot.launch" on Beaglebone blue is required. 
- move_base_remote_gmapping: This launch file is similar to "move_base_remote" but using gmapping for map and locating instead.
- small_robot_configuration: This launch file launch "small_robot_llc/small_robot.launch". This launch file is currently not in use. 

## log
- 2020/11/11: Lack of deadzone detection and racting cause robot jitter.
  - Same day: Add deadzone detection and process algorithm into joy_control_llc_node.
  - fix.
- 2020/11/11: Algrithem relate to turn is not complete yet.
  - Same day: Turn algrithem complete implementation. Test but not benchmark yet.
  - fix
- 2020/11/11: Reaction to timeout of geometry_msg/Twist need to be verified.
  - Same day: `priv_motor_ena` turn to `0` at 99 second after `cmd` stop publishing.
  - 2020/11/18: Add mutex to set and get of `motor_ena` does not solve the problem.
                Will look into timeout detection functions later.
  - 2020/11/25: bug cause by mixing type `float` with `double`.
  - fix
- 2020/11/25: IMU timeout triggered too often. 
  - 2020/11/29: IMU timeout cause by invoking `rc_set_imu_interrupt_func()` before `rc_initialize_imu_dmp()`. Invoke `rc_set_imu_interrupt_func()` after `rc_initialize_imu_dmp()` than the callback works fine.
  - fix
- 2020/11/29: Add odometry message related code. (No need to track)
- 2020/12/02: Odometry will drift in short amount of distance. (At travel distance less than 4m)
  -2020/12/02: Update odometry algorithm to "exact integration" plus "2nd order Runge-Kutta"
  - fix
- 2020/12/05: Change name of `joy_control_llc_node.cpp` to `sr_llc_node.cpp`.
- 2020/12/09: "Corrupted double-linked list" may occurred randomly when killing node `sr_llc_node`.

### Todos
- [x] Create paramete.
- [x] Look into timeout detection function.
- [x] Let IMU online.
- [x] Create launch file.
- [ ] Create config file.
