#ifndef BEECOM_WIFI_TYPE_H
#define BEECOM_WIFI_TYPE_H

#include <beecomint.h>

#define AF_INET 	0

#define SOCK_STREAM 	0
#define SOCK_DGRAM 		1

#define LISTEN_QUEUE 	8


// struct in_addr {
//	uint32_t s_addr;
// };

typedef struct in_addr {
	uint8_t s_addr[16];
}in_addr;

typedef struct BC_Sockaddr {
	sint16_t sin_family;                      /* Address family */
	uint16_t sin_port;       /* Port number */
	in_addr sin_addr;              /* Internet address */
	uint8_t sin_zero[8];         /* Same size as struct sockaddr */
}BC_Sockaddr;

typedef struct BC_SocketData {
	BC_Sockaddr addr;
	bool_t valid;
	uint8_t * buf;
	sint32_t backlog;
}BC_SocketData;

#endif
