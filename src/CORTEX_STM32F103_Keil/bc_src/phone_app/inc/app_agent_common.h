#ifndef APP_AGENT_COMMON_H
#define APP_AGENT_COMMON_H


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "beecomint.h"

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
	uint32_t msg_flag;
	BC_Sockaddr addr;
	bool_t valid;
	// bool_t is_server;
	// bool_t accepted;
	uint32_t wifi_recv_flag;
	uint8_t * buf;
	sint32_t backlog;
	QueueHandle_t queue_handle;
	sint32_t wifi_id;
	sint32_t ipd_size;
}BC_SocketData;

// wifi APIs
// They are similar to linux socket
// just for using the linux net code procedure
// Author: Ansersion
// Date: 2017-03-14
sint32_t BC_Socket(sint32_t family, sint32_t type, sint32_t protocol);
sint32_t BC_Bind(sint32_t sockfd, const BC_Sockaddr * myaddr, uint32_t addrlen);
sint32_t BC_Listen(sint32_t sockfd, sint32_t backlog);
sint32_t BC_Accept(sint32_t sockfd, BC_Sockaddr * cliaddr, uint32_t * addrlen);
sint32_t BC_Connect(sint32_t sockfd, const BC_Sockaddr * servaddr, uint32_t addrlen);
sint32_t BC_Close(sint32_t sockfd);
sint32_t BC_Recv(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags);
sint32_t BC_Send(sint32_t sockfd, const void * buff, uint32_t nbytes, sint32_t flags);

#endif

