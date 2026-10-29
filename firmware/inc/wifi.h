#ifndef WIFI_H
#define WIFI_H

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

#define UDP_PORT 42069

int udp_create_socket(struct sockaddr_in *serv);
int udp_init_client(struct sockaddr_in *client);
bool network_check_port(void);
ssize_t udp_send(int sock, char *data, struct sockaddr_in *client);
ssize_t udp_recv(int sock, char *buf, size_t max_len, struct sockaddr_in
                 *client);

#endif // WIFI_H
