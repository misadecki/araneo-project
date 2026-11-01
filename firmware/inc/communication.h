#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <zephyr/kernel.h>

#define WIFI_STACK_SIZE 4096
#define WIFI_PRIORITY 7
#define CMD_TIMEOUT_MS 500

struct __packed robot_packet {
  float angle[8];
};

struct robot_command {
  float angle[8];
  int64_t stamp_ms;
};

bool get_latest_command(struct robot_command *cmd);
void communication_start_thread(void);

#endif // COMMUNICATION_H
