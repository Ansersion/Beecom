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

#include <bc_type.h>
#include <utils.h>

sint32_t CheckDataFlag(uint8_t * buf, uint32_t buf_size, uint8_t * flag, uint32_t flag_size, bool_t is_end_term)
{
	static sint32_t i = 0;
	// static sint32_t j = 0;
	if(buf_size < flag_size) {
		return BC_ERR;
	}
	if(!buf) {
		return BC_ERR;
	}
	if(!flag) {
		return BC_ERR;
	}
	if(is_end_term) {
		for(i = 0; i < flag_size; i++) {
			if(buf[buf_size - flag_size + i] != flag[i]) {
				return BC_ERR;
			}
		}
	} else {
		for(i = 0; i < flag_size; i++) {
			if(buf[i] != flag[i]) {
				return BC_ERR;
			}
		}
	}
	return BC_OK;
}
