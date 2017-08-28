//   Copyright 2017 Ansersion
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//

#ifndef WIFI_CLBK_H
#define WIFI_CLBK_H

// Beecom headers
#include <wifi_common.h>

typedef enum WIFI_CLBK_CMD_ID {
	/*For wifi configure*/
	WIFI_CLBK_CMD_NONE = 0,
	WIFI_CLBK_CMD_RESET,
	WIFI_CLBK_CMD_SET_MODE,
	WIFI_CLBK_CMD_SET_NET,
	WIFI_CLBK_CMD_SET_MUX,
	WIFI_CLBK_CMD_SET_SERV,
	WIFI_CLBK_CMD_QRY_SR,

	/*For connection*/
	WIFI_CLBK_CMD_QRY_ST,
	WIFI_CLBK_CMD_CLOSE_SOCK,
	WIFI_CLBK_CMD_SEND,
}WIFI_CLBK_CMD_ID;

/*WIFI_CLBK_CMD_RESET*/
typedef struct stWifiClbkResetPara {
	// no use
	uint32_t NO_USE;
	// // uint32_t * Timeout;
}stWifiClbkResetPara;

/*WIFI_CLBK_CMD_SET_MODE*/
typedef struct stWifiClbkSetModePara {
	WIFI_MODE Mode;
	// uint32_t * Timeout;
}stWifiClbkSetModePara;

/*WIFI_CLBK_CMD_SET_NET*/
typedef struct stWifiClbkSetNetPara {
	uint8_t * Ssid;
	uint8_t * Pwd;
	// uint32_t * Timeout;
}stWifiClbkSetNetPara;

/*WIFI_CLBK_CMD_SET_MUX*/
typedef struct stWifiClbkSetMuxPara {
	WIFI_MUX Mux;
	// uint32_t * Timeout;
}stWifiClbkSetMuxPara;

/*WIFI_CLBK_CMD_SET_SERV*/
typedef struct stWifiClbkServPara{
	WIFI_SERVER ServMode;
	uint16_t Port;
	// uint32_t * Timeout;
}stWifiClbkServPara;

/*WIFI_CLBK_CMD_QRY_SR*/
typedef struct stWifiClbkSrPara{
	// no use
	uint32_t NO_USE;
}stWifiClbkSrPara;

/*WIFI_CLBK_CMD_QRY_ST*/
typedef struct stWifiClbkStPara{
	// no use
	uint32_t NO_USE;
}stWifiClbkStPara;

/*WIFI_CLBK_CMD_CLOSE_SOCK*/
typedef struct stWifiClbkClsSockPara{
	sint32_t sockfd;
}stWifiClbkClsSockPara;

/*WIFI_CLBK_CMD_SEND*/
typedef struct stWifiClbkSendPara{
	sint32_t sockfd;
	uint8_t * msg;
	uint32_t size;
}stWifiClbkSendPara;

/*********************/
typedef union unWifiClbkPara {
	stWifiClbkResetPara 	ResetPara;
	stWifiClbkSetModePara 	SetModePara;
	stWifiClbkSetNetPara 	SetNetPara;
	stWifiClbkSetMuxPara 	SetMuxPara;
	stWifiClbkServPara 		ServPara;
	stWifiClbkSrPara 		SrPara;
	stWifiClbkStPara 		StPara;
	stWifiClbkClsSockPara 	ClsSockPara;
	stWifiClbkSendPara 		SendPara;
}unWifiClbkPara;

typedef struct stWifiMsgUnit {
	WIFI_CLBK_CMD_ID WifiClbkCmd;
	unWifiClbkPara ClbkPara;
}stWifiMsgUnit;

// sint32_t CheckWifiMsgUnit(void);

#endif


