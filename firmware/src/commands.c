#include "commands.h"
#include <stdlib.h>
#include "icons.h"
#include "servo.h"
#include "oled.h"

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

int cmd_oled_set_face(const struct shell *sh, int argc, char **argv) {
  if (argc < 1) {
    shell_error(sh, "Usage: face [face_id] <standard | angry | sad | smile | "
                "kidding | eliza | sleep | shocked | excited>");
    return -1;
  }

  icon_id_t icon_id;

  if (icon_id_from_str(argv[1], &icon_id) < 0) {
    shell_error(sh, "Unknown icon.");
    return -2;
  }

  draw_img(get_icon(icon_id));
  return 0;
}

SHELL_CMD_REGISTER(servo, NULL, "Servos commands", cmd_servo_set_angle);
SHELL_CMD_REGISTER(face, NULL, "OLED commands", cmd_oled_set_face);
