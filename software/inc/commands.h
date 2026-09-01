#ifndef COMMANDS_H__
#define COMMANDS_H__

#include <zephyr/shell/shell.h>

int cmd_servo_set_angle(const struct shell *sh, int argc, char **argv);
int cmd_oled_set_face(const struct shell *sh, int argc, char **argv);

#endif // COMMANDS_H__
