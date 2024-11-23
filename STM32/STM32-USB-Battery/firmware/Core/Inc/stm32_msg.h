/*
 * stm32_msg.h
 *
 *  Created on: Apr 7, 2024
 *      Author: opencores
 */

#ifndef INC_STM32_MSG_H_
#define INC_STM32_MSG_H_

#include <stdint.h>

void prevent_push_pack_issue(void);

#pragma pack (push, 1)

#define STM32_MSG_DATA_FLAG 	0x80
#define STM32_MSG_GROUP_SYS 	0x00
#define STM32_MSG_GROUP_INA219	0x01
#define STM32_MSG_GROUP_BC  	0x02

#define STM_MSG_SYS_DATEH	 		0x0000
#define STM_MSG_SYS_DATEL	 		0x0001
#define STM_MSG_SYS_TIMEH	 		0x0002
#define STM_MSG_SYS_TIMEL	 		0x0003
#define STM_MSG_SYS_UPTIME	 		0x0004

#define STM_MSG_INA219_CONF 		0x0000
#define STM_MSG_INA219_SHUNT 		0x0001
#define STM_MSG_INA219_BUS			0x0002
#define STM_MSG_INA219_POWER		0x0003
#define STM_MSG_INA219_CURRENT		0x0004
#define STM_MSG_INA219_CALIBRATION	0x0005

#define STM_MSG_BC_STAT 			0x0000
#define STM_MSG_BC_CONTROL 			0x0001
#define STM_MSG_BC_VSUPPLY 			0x0002

#define STM_MSG_BC_STAT_PG 			0x0001
#define STM_MSG_BC_STAT_STAT_2		0x0002
#define STM_MSG_BC_STAT_STAT_1		0x0004

#define STM_MSG_BC_CONTROL_SEL		0x0001
#define STM_MSG_BC_CONTROL_PROG2	0x0002
#define STM_MSG_BC_CONTROL_CE		0x0004

#define STM32_CMD_STATUS_NO_REPLY		0x00
#define STM32_CMD_STATUS_REPLY_READY	0x01
#define STM32_CMD_STATUS_REPLY_ERROR	0x02
#define STM32_CMD_STATUS_UNKNOWN_CMD	0x03
#define STM32_CMD_STATUS_ERROR		    0x04


typedef struct {
	uint8_t 	cmd_addr_group;
	uint16_t 	cmd_addr;
	uint16_t 	data;
} stm32_cmd_msg_t;

#pragma pack (pop)

typedef struct stm32_msg_handler_s
{
    uint8_t addr_group;																	/**< address group */
    uint8_t (*request_msg)(stm32_cmd_msg_t *request_msg, stm32_cmd_msg_t *reply_msg);	/**< request message handler */
    uint8_t (*command_msg)(stm32_cmd_msg_t *cmd_msg);									/**< command message handler */
} stm32_msg_handler_t;

typedef struct bm_data_s
{
  uint16_t stat;
  uint16_t control;
  uint16_t vsupply;
} bm_data_t;

#endif /* INC_STM32_MSG_H_ */
