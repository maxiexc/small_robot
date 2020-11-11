
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H


/*Predefine macros*/
#define MOTOR_PULSE_PER_REV   2800
#define MOTOR_PULSE_PER_METER 21738
#define TRACK_WIDTH_M         0.125

/*Motor performance parameters*/
#define MOTOR_SPEED_MAX_ABS   8000
#define MOTOR_SPEED_MIN_ABS   100
#define MOTOR_DDUTY_MAX001    (20.0f)     
#define MOTOR_DUTY_MAX_ABS    (1.0f)
#define MOTOR_DUTY_MIN_ABS    (0.15f)

/*Contorller  related parameters*/
#define MOTOR_ERR_THD         2       /*Deadzone for motor controller in PPS*/
#define MOTOR_SPEED_CONTROLLER_KP (0.015f)
#define MOTOR_SPEED_CONTROLLER_KD (0.0f)

#endif