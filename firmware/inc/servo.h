#ifndef SERVO_H__
#define SERVO_H__

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#define MIN_PULSE_US 1000
#define MAX_PULSE_US 2000

#define SERVO_COXA_LEFT_FRONT_OFFSET 0
#define SERVO_COXA_RIGHT_FRONT_OFFSET 0
#define SERVO_COXA_LEFT_BACK_OFFSET 0
#define SERVO_COXA_RIGHT_BACK_OFFSET 0
#define SERVO_TIBIA_LEFT_FRONT_OFFSET 0
#define SERVO_TIBIA_RIGHT_FRONT_OFFSET 0
#define SERVO_TIBIA_LEFT_BACK_OFFSET 0
#define SERVO_TIBIA_RIGHT_BACK_OFFSET 0

typedef enum {
  SERVO_COXA_LEFT_FRONT = 0,
  SERVO_COXA_RIGHT_FRONT,
  SERVO_COXA_LEFT_BACK,
  SERVO_COXA_RIGHT_BACK,
  SERVO_TIBIA_LEFT_FRONT,
  SERVO_TIBIA_RIGHT_FRONT,
  SERVO_TIBIA_LEFT_BACK,
  SERVO_TIBIA_RIGHT_BACK,

  SERVO_COUNT
} servo_id_t;

struct servo_config {
  struct pwm_dt_spec pwm;
  int32_t offset_deg;
  int32_t current_angle;
};

struct servo_arg_map {
  const char *arg;
  servo_id_t id;
};

int servo_init_all(void);
int servo_id_from_str(const char *str, servo_id_t *id);
int servo_init(servo_id_t id);
int servo_set_angle(servo_id_t id, uint32_t angle_deg);
int32_t servo_get_angle(servo_id_t id);

#endif // SERVO_H__
