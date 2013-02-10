#ifndef __AOS_RESOURCE_H_
#define __AOS_RESOURCE_H_

// notes at <https://docs.google.com/document/d/1gzRrVcqL2X9VgNQUI5DrvLVVVziIH7c5ZerATVbiS7U/edit?hl=en_US>

#include <sys/types.h>
#include "shared_mem.h"
#include "binheap.h"
#include "aos_sync.h"
#include "core_lib.h"

#define AOS_RESOURCE_PRIORITY_STACK_LENGTH 50

extern int RESOURCE_KILL_SIGNAL;

/* Return Values
   0  = success
   -1 = you should stop (like if you got killed) (only for request)
   1  = error
*/
int aos_resource_request(aos_resource_entity *local, aos_resource *resource);
int aos_resource_release(aos_resource_entity *local, aos_resource *resource);
int aos_resource_entity_set_parent(aos_resource_entity *local, aos_resource_entity *parent);
aos_resource_entity *aos_resource_entity_create(uint8_t base_priority);
aos_resource *aos_resource_get(uint16_t num);
aos_resource_entity *aos_resource_entity_root_get(void);

#endif
