#include "wifi.h"
#include <zephyr/logging/log.h>
#include <zephyr/net/net_ip.h>

LOG_MODULE_REGISTER(wifi, LOG_LEVEL_INF);

bool network_check_port(void) {
  if (UDP_PORT > 65535 || UDP_PORT < 0) {
    LOG_ERR("Bad port.\n");
    return false;
  }

  return true;
}

int udp_create_socket(struct sockaddr_in *serv) {
  int sock;
  if ((sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP)) < 0) {
    LOG_ERR("Error creating socket: %d\n", errno);
    return -1;
  }

  if (!network_check_port())
    return -1;

  serv->sin_family = NET_AF_INET;
  serv->sin_addr.s_addr = net_htonl(NET_INADDR_ANY);
  serv->sin_port = net_htons(UDP_PORT);

  if (zsock_bind(sock, (struct net_sockaddr*)serv, sizeof(*serv)) < 0) {
    LOG_ERR("Error binding socket: %d\n", errno);
    zsock_close(sock);
    return -1;
  }

  return sock;
}

int udp_init_client(struct sockaddr_in *client) {
  if (!client)
    return -1;

  memset(client, 0, sizeof(struct sockaddr_in));
  client->sin_family = NET_AF_INET;
  return 0;
}
