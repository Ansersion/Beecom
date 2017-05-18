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
	WIFI_CLBK_CMD_NONE = 0,
	WIFI_CLBK_CMD_RESET,
	WIFI_CLBK_CMD_SET_MODE,
	WIFI_CLBK_CMD_SET_NET,
	WIFI_CLBK_CMD_SET_MUX,
	WIFI_CLBK_CMD_SET_SERV,
}WIFI_CLBK_CMD_ID;

typedef struct stWifiClbkResetPara {
	// no use
	uint32_t NO_USE;
	// // uint32_t * Timeout;
}stWifiClbkResetPara;

typedef struct stWifiClbkSetModePara {
	WIFI_MODE Mode;
	// uint32_t * Timeout;
}stWifiClbkSetModePara;

typedef struct stWifiClbkSetNetPara {
	uint8_t * Ssid;
	uint8_t * Pwd;
	// uint32_t * Timeout;
}stWifiClbkSetNetPara;

typedef struct stWifiClbkSetMuxPara {
	WIFI_MUX Mux;
	// uint32_t * Timeout;
}stWifiClbkSetMuxPara;

typedef struct stWifiClbkServPara{
	WIFI_SERVER ServMode;
	uint16_t Port;
	// uint32_t * Timeout;
}stWifiClbkServPara;

typedef union unWifiClbkPara {
	stWifiClbkResetPara 	ResetPara;
	stWifiClbkSetModePara 	SetModePara;
	stWifiClbkSetNetPara 	SetNetPara;
	stWifiClbkSetMuxPara 	SetMuxPara;
	stWifiClbkServPara 		ServPara;
}unWifiClbkPara;

typedef struct stWifiMsgUnit {
	WIFI_CLBK_CMD_ID WifiClbkCmd;
	unWifiClbkPara ClbkPara;
}stWifiMsgUnit;

sint32_t CheckWifiMsgUnit(void);

#endif


