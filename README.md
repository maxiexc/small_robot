# Small Robot

## Package: Small Robot LLC
This is a ROS package contain low level control algrithem of small robot.
This package is designed for Beaglebone blue

### Subsrcibtion topics
cmd_vel: geometry/Twist

### Publish topics
No at the moment

### Actions
Control motor 1 and 2 acoording to command from cmd_vel.

### Parameters
No at the moment

### log
- 2020/11/11: Lack of deadzone detection and racting cause robot jitter.
  - Same day: Add deadzone detection and process algorithm into joy_control_llc_node.
- 2020/11/11: Algrithem relate to turn is not complete yet.
  - Same day: Turn algrithem complete implementation. Test but not benchmark yet.
- 2020/11/11: Reaction to timeout of geometry_msg/Twist need to be verified.
  - Same day: `priv_motor_ena` turn to `0` at 99 second after `cmd` stop publishing.

### Todos
- [ ] Nothing yet
