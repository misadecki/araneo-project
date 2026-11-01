#include "communication.h"
#include "wifi.h"
#include <sys/errno.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(communication, LOG_LEVEL_INF);

#define MAX_ANGLE_DEG 180.0f

static struct robot_command latest_cmd;
static struct k_thread wifi_thread_data;
K_THREAD_STACK_DEFINE(wifi_stack_area, WIFI_STACK_SIZE);
K_MUTEX_DEFINE(cmd_lock);

static bool packet_valid(const struct robot_packet *pckt) {
  for (int i = 0; i < 8; ++i) {
    if (!isfinite(pckt->angle[i]) || fabsf(pckt->angle[i]) > MAX_ANGLE_DEG) {
      LOG_ERR("Bad data.");
      return -false;
    }
  }

  return true;
}

static void wifi_thread_entry(void *pa1, void *p2, void *p3) {
  struct net_sockaddr_in serv_addr;
  int sock = udp_create_socket(&serv_addr);
  if (sock < 0) {
    LOG_ERR("Socket creation failed.");
    return;
  }

  struct robot_packet pckt;
  struct net_sockaddr_in client_addr;

  if (udp_init_client(&client_addr) < 0)
    return;

  while (1) {
    ssize_t received = udp_recv(sock, (char *)&pckt, sizeof(pckt),
                                &client_addr);

    if (received == sizeof(pckt)) {
      if (!packet_valid(&pckt)) {
        LOG_WRN("Invalid packet.");
        continue;
      }
      k_mutex_lock(&cmd_lock, K_FOREVER);
        memcpy(&latest_cmd.angle, pckt.angle, sizeof(pckt.angle));
        latest_cmd.stamp_ms = k_uptime_get();
      k_mutex_unlock(&cmd_lock);
      LOG_INF("RX: %.1f %.1f", (double)pckt.angle[0], (double)pckt.angle[1]);
    } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      LOG_WRN("Receive error: %d", errno);
      k_sleep(K_MSEC(100));
    }
  }
}

bool get_latest_command(struct robot_command *cmd) {
  k_mutex_lock(&cmd_lock, K_FOREVER);
    memcpy(cmd, &latest_cmd, sizeof(struct robot_command));
  k_mutex_unlock(&cmd_lock);

  if (cmd->stamp_ms == 0)
    return false;

  return (k_uptime_get() - cmd->stamp_ms) < CMD_TIMEOUT_MS;
}

void communication_start_thread(void) {
  k_thread_create(&wifi_thread_data, wifi_stack_area,
                  K_THREAD_STACK_SIZEOF(wifi_stack_area), wifi_thread_entry,
                  NULL, NULL, NULL, WIFI_PRIORITY, 0, K_NO_WAIT);
}

