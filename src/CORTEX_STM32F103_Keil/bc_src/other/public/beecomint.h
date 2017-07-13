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

#ifndef BEECOMINT_H
#define BEECOMINT_H

#define BC_FALSE 		pdFALSE
#define BC_TRUE 		pdTRUE

#define BC_OK 	0
#define BC_ERR 	1

typedef char 	bool_t;

typedef unsigned char 	uint8_t;
typedef unsigned short 	uint16_t;
typedef unsigned int 	uint32_t;

typedef char  	sint8_t;
typedef short 	sint16_t;
typedef int 	sint32_t;

typedef unsigned int   OS_STK;			// Each stack entry is 32-bit wide(OS Stack)

#endif
