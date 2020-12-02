# Small Robot

## Package: Small Robot LLC
This is a ROS package contain low level control algrithem of small robot.
This package is designed for Beaglebone blue

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

### log
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
- 2020/11/29: Add odometry message related code.
- 2020/12/02: Odometry will drift in short amount of distance. (less than 4m)

### Todos
- [x] Create paramete.
- [x] Look into timeout detection function.
- [x] Let IMU online.
- [ ] Create launch file.
- [ ] Create config file.
