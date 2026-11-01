#include "wifi.h"
#include "secrets.h"
#include <sys/errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

LOG_MODULE_REGISTER(wifi, LOG_LEVEL_INF);
BUILD_ASSERT(UDP_PORT > 0 && UDP_PORT <= 65535, "Bad UDP port.");
static K_SEM_DEFINE(ip_sem, 0, 1);
static struct net_mgmt_event_callback ip_cb;
static void ip_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
                       struct net_if *iface) {
  if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
    k_sem_give(&ip_sem);
  }
}

int wifi_init(void) {
  struct net_if *iface = net_if_get_first_wifi();
  if (!iface) {
    LOG_ERR("No WiFi interface.");
    return -ENODEV;
  }

  net_mgmt_init_event_callback(&ip_cb, ip_handler, NET_EVENT_IPV4_ADDR_ADD);
  net_mgmt_add_event_callback(&ip_cb);

  struct wifi_connect_req_params params = {
    .ssid = (const uint8_t*) WIFI_SSID,
    .ssid_length = strlen(WIFI_SSID),
    .psk = (const uint8_t*) WIFI_PASS,
    .psk_length = strlen(WIFI_PASS),
    .security = WIFI_SECURITY_TYPE_PSK,
    .band = WIFI_FREQ_BAND_UNKNOWN,
    .channel = WIFI_CHANNEL_ANY,
    .mfp = WIFI_MFP_OPTIONAL
  };

  int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
  if (ret) {
    LOG_ERR("WiFi connect failed: %d", ret);
    return ret;
  }

  if (k_sem_take(&ip_sem, K_SECONDS(30)) != 0) {
    LOG_ERR("Timeout waiting for IP.");
    return -ETIMEDOUT;
  }

  LOG_INF("Successfully connected to WiFi.");
  set_ip_address();
  return 0;
}

void set_ip_address(void) {
  struct net_if *iface = net_if_get_first_wifi();
  struct net_in_addr *addr;

  if (iface) {
    addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
  } else {
    addr = NULL;
  }

  if (!addr) {
    LOG_WRN("No IPv4 address");
    return;
  }

  char ip_str[NET_IPV4_ADDR_LEN];
  net_addr_ntop(NET_AF_INET, addr, ip_str, sizeof(ip_str));
  LOG_INF("Pico IP address: %s", ip_str);
}

bool network_is_connected(void) {
  struct net_if *iface = net_if_get_first_wifi();
  if (!iface) {
    LOG_ERR("Can't get default network interface.");
    return false;
  }

  if (!net_if_is_up(iface)) {
    LOG_ERR("Network isn't up.");
    return false;
  }

  return net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL;
}

int udp_create_socket(struct net_sockaddr_in *serv) {
  int sock;
  if ((sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP)) < 0) {
    LOG_ERR("Error creating socket: %d\n", errno);
    return -1;
  }

  memset(serv, 0, sizeof(*serv));
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

int udp_init_client(struct net_sockaddr_in *client) {
  if (!client)
    return -1;

  memset(client, 0, sizeof(*client));
  client->sin_family = NET_AF_INET;
  return 0;
}

ssize_t udp_send(int sock, char *data, size_t len, struct net_sockaddr_in *client) {
  if (!data || sock < 0 || !client)
    return -1;

  return zsock_sendto(sock, data, len, 0, (struct net_sockaddr*)client,
                      sizeof(*client));
}

ssize_t udp_recv(int sock, char *buf, size_t max_len, struct net_sockaddr_in
                 *client) {
  if (sock < 0 || !buf || !client)
    return -1;

  socklen_t client_addr_len = sizeof(*client);
  return zsock_recvfrom(sock, buf, max_len, 0, (struct net_sockaddr*)client, &client_addr_len);

}
