#include <zephyr/logging/log.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>
#include <quadruped_msgs/msg/servo_cmd.h>
#include "servo.h"
#include "micro_ros_client.h"

#define MICRO_ROS_STACK_SIZE 16384
#define MICRO_ROS_PRIO 5

LOG_MODULE_REGISTER(micro_ros_client, LOG_LEVEL_INF);
K_MUTEX_DEFINE(cmd_lock);

static quadruped_msgs__msg__ServoCmd servo_msg;
static float latest[SERVO_COUNT];

static void servo_callback(const void* msg_in) {
  const quadruped_msgs__msg__ServoCmd* m = (const
                                            quadruped_msgs__msg__ServoCmd*)msg_in;
  k_mutex_lock(&cmd_lock, K_FOREVER);
  memcpy(latest, m->angle_rad, sizeof(latest));
  k_mutex_unlock(&cmd_lock);
}

int micro_ros_get_cmd(float out[SERVO_COUNT]) {
  k_mutex_lock(&cmd_lock, K_FOREVER);
  memcpy(out, latest, sizeof(latest));
  k_mutex_unlock(&cmd_lock);

  return 0;
}

static void micro_ros_thread(void *a, void *b, void *c) {
  rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
                                (void*)&default_params, zephyr_transport_open,
                                zephyr_transport_close, zephyr_transport_write,
                                zephyr_transport_read);

  while (1) {
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      k_msleep(500);
    }

    LOG_INF("Agent found.");

    rcl_allocator_t alloc = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_subscription_t sub;
    rclc_executor_t exec;

    if (rclc_support_init(&support, 0, NULL, &alloc) != RCL_RET_OK) {
      k_msleep(1000);
      continue;
    }
    rclc_node_init_default(&node, "araneo_firmware", "", &support);
    rclc_subscription_init_default(&sub, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(quadruped_msgs, msg,
                                                               ServoCmd), "/servo_cmd");
    rclc_executor_init(&exec, &support.context, 1, &alloc);
    rclc_executor_add_subscription(&exec, &sub, &servo_msg, servo_callback,
                                   ON_NEW_DATA);

    int missed = 0;
    while (missed < 5) {
      rclc_executor_spin_some(&exec, RCL_MS_TO_NS(10));
      if (rmw_uros_ping_agent(50, 1) != RCL_RET_OK) {
        ++missed;
      } else {
        missed = 0;
      }
      k_msleep(1);
    }

    LOG_WRN("Agent lost. Reconnecting..");
    if (rclc_executor_fini(&exec) != RCL_RET_OK)
      LOG_ERR("executor deinit failed");
    if (rcl_subscription_fini(&sub, &node) != RCL_RET_OK)
      LOG_ERR("sub deinit failed");
    if (rcl_node_fini(&node) != RCL_RET_OK)
      LOG_ERR("node deinit failed");
    if (rclc_support_fini(&support) != RCL_RET_OK)
      LOG_ERR("support deinit failed");
  }
}

K_THREAD_DEFINE(micro_ros_thread_id, MICRO_ROS_STACK_SIZE, micro_ros_thread,
                NULL, NULL, NULL, MICRO_ROS_PRIO, 0, 0);
