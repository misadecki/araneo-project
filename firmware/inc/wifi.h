#ifndef WIFI_H
#define WIFI_H

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

#define UDP_PORT 42069

int wifi_init(void);
bool network_is_connected(void);
void set_ip_address(void);
int udp_create_socket(struct net_sockaddr_in *serv);
int udp_init_client(struct net_sockaddr_in *client);
ssize_t udp_send(int sock, char *data, size_t len, struct net_sockaddr_in
                 *client);
ssize_t udp_recv(int sock, char *buf, size_t max_len, struct net_sockaddr_in
                 *client);

#endif // WIFI_H
