#ifndef __AOS_RESOURCE_INTERNAL_H_
#define __AOS_RESOURCE_INTERNAL_H_

#include "resource.h"

#define AOS_RESOURCE_STATES_PER_BYTE 4
#define AOS_RESOURCE_STATE_SHIFTER(num) ((num % AOS_RESOURCE_STATES_PER_BYTE) * (8 / AOS_RESOURCE_STATES_PER_BYTE))
#define AOS_RESOURCE_STATE_GET(num, entity) (entity->state[num / AOS_RESOURCE_STATES_PER_BYTE] >> AOS_RESOURCE_STATE_SHIFTER(num))
#define AOS_RESOURCE_STATE_SET_ON(mask, num, entity) (entity->state[num / AOS_RESOURCE_STATES_PER_BYTE] |= (mask << AOS_RESOURCE_STATE_SHIFTER(num)))
#define AOS_RESOURCE_STATE_SET_OFF(mask, num, entity) (entity->state[num / AOS_RESOURCE_STATES_PER_BYTE] &= ~(mask << AOS_RESOURCE_STATE_SHIFTER(num)))
#define AOS_RESOURCE_STATE_GET_HAS_IT(num, entity) (AOS_RESOURCE_STATE_GET_WANTS_IT(num, entity) || AOS_RESOURCE_STATE_GET_HAS_PASSED_DOWN(num, entity))
#define AOS_RESOURCE_STATE_WANTS_IT 0x01
#define AOS_RESOURCE_STATE_HAS_PASSED_DOWN 0x02
#define AOS_RESOURCE_STATE_GET_WANTS_IT(num, entity) (AOS_RESOURCE_STATE_GET(num, entity) & AOS_RESOURCE_STATE_WANTS_IT)
#define AOS_RESOURCE_STATE_GET_HAS_PASSED_DOWN(num, entity) (AOS_RESOURCE_STATE_GET(num, entity) & AOS_RESOURCE_STATE_HAS_PASSED_DOWN)

struct aos_resource_entity_t{
	aos_resource_entity *parent;
	uint8_t state[(AOS_RESOURCE_NUM + (AOS_RESOURCE_STATES_PER_BYTE - 1)) / AOS_RESOURCE_STATES_PER_BYTE];
	pid_t root_action;
	uint8_t base_priority, priority;
};

inline void aos_resource_init(uint16_t num);

#endif

