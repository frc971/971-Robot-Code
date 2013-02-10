//#define TESTING_ASSERT(...)
#define TESTING_ASSERT(cond, desc, args...) if(!(cond)){fprintf(stderr, "error: " desc " at " __FILE__ ": %d\n", ##args, __LINE__);}
#define TESTING_ASSERT_RETURN(cond, desc, args...) TESTING_ASSERT(cond, desc, ##args); if(!(cond)){return 1;}
// leave TESTING_ASSERT_RETURN (segfaults result otherwise)

#include "resource_internal.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

int RESOURCE_KILL_SIGNAL;

__attribute__((constructor)) void __constructor(){
	RESOURCE_KILL_SIGNAL = SIGRTMIN + 5;
}

void aos_resource_init(uint16_t num){
	aos_resource *resource = aos_resource_get(num);
	resource->num = num;
	resource->resource_mutex = 0;
	resource->modify = 0;
	resource->request = 0;
	resource->kill = 0;
	resource->owner = 0;
	resource->priorities = shm_malloc(sizeof(struct HeapStruct) + AOS_RESOURCE_PRIORITY_STACK_LENGTH * sizeof(uint8_t));
	resource->priorities->Elements = (uint8_t *)((uintptr_t)resource->priorities + sizeof(struct HeapStruct)); // only do 1 shm_malloc (directly above)
	Initialize(AOS_RESOURCE_PRIORITY_STACK_LENGTH, resource->priorities);
	AOS_RESOURCE_STATE_SET_ON(AOS_RESOURCE_STATE_WANTS_IT, num, aos_resource_entity_root_get());
}
void aos_resource_entity_root_create(){
	global_core->mem_struct->resources.root = aos_resource_entity_create(0);
}
inline aos_resource_entity *aos_resource_entity_root_get(){
	return global_core->mem_struct->resources.root;
}
inline aos_resource *aos_resource_get(uint16_t num){
	return &global_core->mem_struct->resources.resources[num];
}

aos_resource_entity *aos_resource_entity_create(uint8_t base_priority){
	aos_resource_entity *local = shm_malloc(sizeof(aos_resource_entity));
	memset(local->state, 0x00, sizeof(local->state));
	local->base_priority = base_priority;
	local->parent = NULL; // for the root entity
	return local;
}
int aos_resource_entity_set_parent(aos_resource_entity *local, aos_resource_entity *parent){
	TESTING_ASSERT_RETURN(local != NULL, "do not have a local entity");
	TESTING_ASSERT_RETURN(parent != local, "can't set parent to self");
	TESTING_ASSERT_RETURN(parent != NULL, "have to have a parent to set to");
	local->parent = parent;
	if(parent->parent == NULL){
		local->root_action = getpid();
	}else{
		local->root_action = parent->root_action;
	}
	if(parent->priority > local->base_priority){
		local->priority = parent->priority;
	}else{
		local->priority = local->base_priority;
	}
	if(local->state[0] != 0 || memcmp(local->state, local->state + 1, sizeof(local->state) - 1)){ // if it's not all 0s
		TESTING_ASSERT(0, "local->state isn't all 0s when changing parents (fixing it)");
		memset(local->state, 0x00, sizeof(local->state));
	}
	return 0;
}

void aos_resource_kill(pid_t action){
	union sigval sival;
	sival.sival_int = 0;
	if(sigqueue(action, RESOURCE_KILL_SIGNAL, sival) < 0){ // if sending the signal failed
		fprintf(stderr, "sigqueue RESOURCE_KILL_SIGNAL (which is %d) with pid %d failed with errno %d ", RESOURCE_KILL_SIGNAL, action, errno);
		perror(NULL);
	}
}
int aos_resource_request(aos_resource_entity *local, aos_resource *resource){
	TESTING_ASSERT_RETURN(local != NULL, "do not have a local entity");
	
	if(mutex_lock(&resource->request))
		return 1;
	if(mutex_lock(&resource->kill)){
		mutex_unlock(&resource->request);
		return 1;
	}
	if(mutex_lock(&resource->modify)){
		mutex_unlock(&resource->kill);
		mutex_unlock(&resource->request);
		return 1;
	}

	aos_resource_entity *c = local;
	while(c->parent != NULL && !AOS_RESOURCE_STATE_GET_HAS_IT(resource->num, c)){
		c = c->parent;
	}
	TESTING_ASSERT((c->parent == NULL) == (c == aos_resource_entity_root_get()), "found a resource with no parent that isn't the root")
	if(c->parent == NULL && !AOS_RESOURCE_STATE_GET_WANTS_IT(resource->num, c)){ // if c is the root entity and doesn't want it
		TESTING_ASSERT(0, "root entity does not want resource %d (will fix it)", resource->num);
		*((int *)NULL) = 0;
		AOS_RESOURCE_STATE_SET_ON(AOS_RESOURCE_STATE_WANTS_IT, resource->num, c);
	}
	uint8_t locked_resource_mutex = 0;
	if(AOS_RESOURCE_STATE_GET_HAS_PASSED_DOWN(resource->num, c)){
		if(c->parent == NULL){
			if(GetMin(resource->priorities) >= local->priority){
				mutex_unlock(&resource->modify);
				aos_resource_kill(resource->owner);
				if(mutex_lock(&resource->kill)){ // got released by one that got killed (after unlocking resource_mutex)
					mutex_unlock(&resource->resource_mutex);
					mutex_unlock(&resource->request);
					return 1;
				}
				if(mutex_lock(&resource->resource_mutex)){ // wait for the other process to release it
					mutex_unlock(&resource->request);
					return 1;
				}
				locked_resource_mutex = 1;
				if(mutex_lock(&resource->modify)){
					mutex_unlock(&resource->request);
					return 1;
				}
			}else{
				mutex_unlock(&resource->modify);
				mutex_unlock(&resource->request);
				return -1;
			}
		}else{
			fprintf(stderr, "PROGRAMMER ERROR!!!!! 2 sub-actions both requested resource %d!!! stopping the root action\n", resource->num);
			mutex_unlock(&resource->modify);
			mutex_unlock(&resource->request);
			return -1;
		}
	}
	AOS_RESOURCE_STATE_SET_ON(AOS_RESOURCE_STATE_WANTS_IT, resource->num, local);
	aos_resource_entity *c2 = local;
	do{
		c2 = c2->parent;
		TESTING_ASSERT_RETURN(c2 != NULL, "couldn't find the parent that has resource %d", resource->num)
		AOS_RESOURCE_STATE_SET_ON(AOS_RESOURCE_STATE_HAS_PASSED_DOWN, resource->num, c2);
	} while(c2 != c);
	TESTING_ASSERT(c2 == c, "c2 != c");
	if(c->parent == NULL){ // if you found the root entity
		resource->owner = local->root_action;
		if(!locked_resource_mutex){
			int rv = mutex_trylock(&resource->resource_mutex); // don't deadlock if somebody died or whatever
			TESTING_ASSERT(rv == 0, "the resource_mutex was already locked when getting %d from the root", resource->num);
		}
	}else{
		TESTING_ASSERT(resource->owner == local->root_action, "my action chain has resource %d, but my chain's root(%d) isn't the owner(%d)", resource->num, local->root_action, resource->owner);
		TESTING_ASSERT(mutex_trylock(&resource->resource_mutex) != 0, "my action has the resource_mutex for %d, but the resource_mutex wasn't already locked", resource->num);
	}
	if(Insert(local->priority, resource->priorities) < 0){
		fprintf(stderr, "BAD NEWS: ran out of space on the priority heap for resource %d. Increase the size of AOS_RESOURCE_PRIORITY_STACK_LENGTH in resource.h\n", resource->num);
		mutex_unlock(&resource->modify);
		mutex_unlock(&resource->request);
		return -1;
	}
	mutex_unlock(&resource->modify);
	mutex_unlock(&resource->kill);
	mutex_unlock(&resource->request);
	return 0;
}

int aos_resource_release(aos_resource_entity *local, aos_resource *resource){
	TESTING_ASSERT_RETURN(local != NULL, "do not have a local entity");
	
	if(mutex_lock(&resource->modify)){
		return 1;
	}

	AOS_RESOURCE_STATE_SET_OFF(AOS_RESOURCE_STATE_WANTS_IT, resource->num, local);
	if(!AOS_RESOURCE_STATE_GET_HAS_PASSED_DOWN(resource->num, local)){ // if we're actually supposed to go release it
		aos_resource_entity *c = local;
		while(c->parent != NULL && !AOS_RESOURCE_STATE_GET_WANTS_IT(resource->num, c)){
			AOS_RESOURCE_STATE_SET_OFF(AOS_RESOURCE_STATE_HAS_PASSED_DOWN, resource->num, c);
			c = c->parent;
		}
		if(c->parent == NULL && !AOS_RESOURCE_STATE_GET_WANTS_IT(resource->num, c)){ // if c is the root entity and doesn't want it
			TESTING_ASSERT(0, "root entity does not want resource %d (will fix it)", resource->num);
			AOS_RESOURCE_STATE_SET_ON(AOS_RESOURCE_STATE_WANTS_IT, resource->num, c);
		}
		AOS_RESOURCE_STATE_SET_OFF(AOS_RESOURCE_STATE_HAS_PASSED_DOWN, resource->num, c);
		Remove(local->priority, resource->priorities);
		TESTING_ASSERT(local->root_action == resource->owner, "freeing a resource (%d) whose owner(%d) isn't my chain's root(%d)", resource->num, resource->owner, local->root_action);
		if(c->parent == NULL){ // if you gave it back to the root entity (c)
			TESTING_ASSERT(IsEmpty(resource->priorities), "priority stack isn't empty (size=%d)", GetSize(resource->priorities));
			resource->owner = 0;
			mutex_unlock(&resource->resource_mutex);
			mutex_unlock(&resource->kill);
		}
	} // else has passed it down
	mutex_unlock(&resource->modify);
	return 0;
}

