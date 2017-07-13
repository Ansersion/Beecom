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

#ifndef TERMINAL_CLBK_H
#define TERMINAL_CLBK_H

// Beecom headers

#include <beecomint.h>

typedef enum TERM_CLBK_CMD_ID {
	/*For terminal configure*/
	TERM_CLBK_CMD_NONE = 0,
	TERM_CLBK_CMD_PRINTF,
}TERM_CLBK_CMD_ID;

/*TERM_CLBK_CMD_PRINTF*/
typedef struct stTermClbkPrintfPara {
	uint8_t * msg;
}stTermClbkPrintfPara;

/*********************/
typedef union unTermClbkPara {
	stTermClbkPrintfPara 	PrintfPara;
}unTermClbkPara;

typedef struct stTermMsgUnit {
	TERM_CLBK_CMD_ID TermClbkCmd;
	unTermClbkPara ClbkPara;
}stTermMsgUnit;

sint32_t CheckTermMsgUnit(void);

#endif


