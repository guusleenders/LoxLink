#ifndef LAN_H
#define LAN_H

#include "lan_config.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t gLan_net_buf[]; // Shared receive/transmit buffer
extern uint8_t gLan_MAC_address[6];
extern uint32_t gLan_ip_addr;    // != 0 => own IPv4 address
extern uint32_t gLan_ip_mask;    // netmask
extern uint32_t gLan_ip_gateway; // gateway/router

/***
 *  Compile-time check for macros
 ***/
#define COMPILE_CHECK(x)    \
  {                         \
    struct _CC {            \
      char a[(x) ? 1 : -1]; \
    };                      \
  }

/***
 *  Big-Endian conversion
 ***/
#define htons(a) ((((a) >> 8) & 0xff) | (((a) << 8) & 0xff00))
#define ntohs(a) htons(a)

#define htonl(a) ((((a) >> 24) & 0xff) | (((a) >> 8) & 0xff00) | (((a & 0xffffff) << 8) & 0xff0000) | (((a & 0xff) << 24) & 0xff000000))
#define ntohl(a) htonl(a)

#define inet_addr(a, b, c, d) (((uint32_t)a) | ((uint32_t)b << 8) | ((uint32_t)c << 16) | ((uint32_t)d << 24))
#define ipv4(a) ((a[3] << 24) | (a[2] << 16) | (a[1] << 8) | (a[0] << 0))

void lan_init(void); // initialize at boot time, request DHCP, if enabled
void lan_poll(void); // called in a loop to check for new packages, DHCP leases, etc

#ifdef __cplusplus
}
#endif

#endif