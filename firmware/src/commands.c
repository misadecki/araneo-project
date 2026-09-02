#include "commands.h"
#include <stdlib.h>
#include "servo.h"

int cmd_servo_set_angle(const struct shell *sh, int argc, char **argv) {
  if (argc < 3) {
    shell_error(sh, "Usage: servo [servo_id] <(coxa | tibia)_(lf | rf | lb | "
                "lf)> [angle] <0-180>");
    shell_error(sh, "Example: servo coxa_lb 120");
    return -1;
  }

  const char *servo_arg = argv[1];
  servo_id_t servo_id;

  if (servo_id_from_str(argv[1], &servo_id) < 0) {
    shell_error(sh, "Unknown servo.");
    return -2;
  }

  char *end;
  long angle = strtol(argv[2], &end, 10);

  if (*end != '\0' || angle < 0 || angle > 180) {
    shell_error(sh, "Invalid angle. Use angle in interval [0-180].");
    return -3;
  }

  int ret = servo_set_angle(servo_id, (int32_t)angle);

  if (ret < 0) {
    shell_error(sh, "Error (%d) setting angle %ld for servo %s.", ret, angle, servo_arg);
    return ret;
  }

  return 0;
}

SHELL_CMD_REGISTER(servo, NULL, "Servos commands", cmd_servo_set_angle);
