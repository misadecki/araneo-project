#ifndef MICRO_ROS_CLIENT_H
#define MICRO_ROS_CLIENT_H

#include <zephyr/kernel.h>
#include "servo.h"

int micro_ros_get_cmd(float out[SERVO_COUNT], k_timeout_t timeout);

#endif // MICRO_ROS_CLIENT_H
