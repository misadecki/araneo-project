#include "servo.h"
#include <zephyr/logging/log.h>
#include <strings.h>

LOG_MODULE_REGISTER(servo, LOG_LEVEL_INF);

static struct servo_arg_map servo_args[] = {
  {"coxa_lf", SERVO_COXA_LEFT_FRONT},
  {"coxa_lb", SERVO_COXA_LEFT_BACK},
  {"coxa_rf", SERVO_COXA_RIGHT_FRONT},
  {"coxa_rb", SERVO_COXA_RIGHT_BACK},
  {"tibia_lf", SERVO_TIBIA_LEFT_FRONT},
  {"tibia_lb", SERVO_TIBIA_LEFT_BACK},
  {"tibia_rf", SERVO_TIBIA_RIGHT_FRONT},
  {"tibia_rb", SERVO_TIBIA_RIGHT_BACK},
};

static struct servo_config servos[SERVO_COUNT] = {
  [SERVO_COXA_LEFT_BACK] = {
    .pwm = PWM_DT_SPEC_GET(DT_ALIAS(servolf)),
    .current_angle = 90,
    .offset_deg = SERVO_COXA_LEFT_BACK_OFFSET
  },
};

int servo_id_from_str(const char *str, servo_id_t *id) {
  if (str == NULL || id == NULL)
    return -1;

  for (size_t i = 0; i < sizeof(servo_args) / sizeof(servo_args[0]); ++i) {
    if (strcasecmp(str, servo_args[i].arg) == 0) {
      *id = servo_args[i].id;
      return 0;
    }
  }

  return -2;
}

int servo_get_angle(servo_id_t id) {
  if (id >= SERVO_COUNT)
    return -1;

  return servos[id].current_angle;
}

int servo_init(servo_id_t id) {
  if (!pwm_is_ready_dt(&servos[id].pwm)) {
    LOG_ERR("PWM for %d servo isn't ready.\n", id);
    return -1;
  }

  return 0;
}

int servo_init_all(void) {
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    if (servo_init(i) < 0)
      return -1;

    servo_set_angle((servo_id_t)i, 90);
  }

  LOG_INF("Initialized %d servos.\n", SERVO_COUNT);
  return 0;
}

int servo_set_angle(servo_id_t id, uint32_t angle_deg) {
  if (id >= SERVO_COUNT || !pwm_is_ready_dt(&servos[id].pwm))
    return -1;

  struct servo_config *s = &servos[id];

  if (angle_deg < 0)
    angle_deg = 0;

  if (angle_deg > 180)
    angle_deg = 180;

  int32_t real_angle_deg = angle_deg + s->offset_deg;

  if (real_angle_deg < 0)
    real_angle_deg = 0;

  if (real_angle_deg > 180)
    real_angle_deg = 180;

  uint32_t pulse_us = MIN_PULSE_US + ((MAX_PULSE_US - MIN_PULSE_US) *
    real_angle_deg / 180);

  int ret = pwm_set_pulse_dt(&s->pwm, PWM_USEC(pulse_us));

  if (ret < 0) {
    LOG_ERR("Failed to set PWM for servo %d (%d).\n", id, ret);
    return ret;
  }

  s->current_angle = angle_deg;
  return 0;
}
