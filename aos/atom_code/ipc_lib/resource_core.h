#ifndef AOS_ATOM_CODE_IPC_LIB_RESOURCE_CORE_H_
#define AOS_ATOM_CODE_IPC_LIB_RESOURCE_CORE_H_
// this file has to be separate due to #include order dependencies

#include "aos/atom_code/ipc_lib/shared_mem.h"
#include "aos/atom_code/ipc_lib/binheap.h"
#include "aos/ResourceList.h"

typedef struct aos_resource_t{
	mutex resource_mutex; // gets locked whenever resource is taken from root entity and unlocked when its given back
	mutex modify; // needs to be locked while somebody is requesting or releasing this resource
	mutex request; // needs to be locked while somebody is requesting this resource (including possibly waiting for somebody else to release it after being killed)
	mutex kill; // gets locked while somebody is dying for this resource (gets unlocked whenever this resource gets given back to the root entity)
	pid_t owner;
	PriorityQueue priorities; // lower number = higher priority

	uint16_t num;
} aos_resource;
typedef struct aos_resource_entity_t aos_resource_entity;
typedef struct aos_resource_list_t {
	aos_resource resources[AOS_RESOURCE_NUM];
	aos_resource_entity *root;
} aos_resource_list;
void aos_resource_init(uint16_t num);
void aos_resource_entity_root_create(void);

#endif

