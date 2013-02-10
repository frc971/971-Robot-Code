#define AOS_ResourceAction_t_HEADER_FRAG int DoSub(uint8_t slot, int in_number);

#include "ResourceAction.h"
#include "ResourceActionChild.h"
extern "C"{
#include <resource_internal.h>
}

int ResourceAction_t::DoSub(uint8_t slot, int in_number){
	if(slot & 0x01)
		if(TryRequestResource(test_resource1)){
			PostStatus(in_number + 1);
			return 0;
		}
	if(slot & 0x02)
		ReleaseResource(test_resource1);
	if(slot & 0x04){
		if(!ResourceActionChild.Execute(slot << 3).success){
			PostStatus(in_number + 2);
			return 0;
		}
	}
	return in_number + ((slot & (0x01 | 0x02)) ? 1 : 0) + ((slot & 0x04) ? 1 : 0);
}
void ResourceAction_t::DoAction(uint8_t first, uint8_t second, int number, uint8_t state_to_set){
	printf("start of ResourceAction.DoAction\n");
	AOS_RESOURCE_STATE_SET_ON((AOS_RESOURCE_STATE_WANTS_IT | AOS_RESOURCE_STATE_HAS_PASSED_DOWN) & state_to_set, test_resource1, ::AsyncActionStatics::resource_entity);
	AOS_RESOURCE_STATE_SET_OFF((AOS_RESOURCE_STATE_WANTS_IT | AOS_RESOURCE_STATE_HAS_PASSED_DOWN) & state_to_set, test_resource1, ::AsyncActionStatics::resource_entity);
	AOS_RESOURCE_STATE_SET_ON((AOS_RESOURCE_STATE_WANTS_IT | AOS_RESOURCE_STATE_HAS_PASSED_DOWN) & (state_to_set >> 2), test_resource1, aos_resource_entity_root_get());
	AOS_RESOURCE_STATE_SET_OFF((AOS_RESOURCE_STATE_WANTS_IT | AOS_RESOURCE_STATE_HAS_PASSED_DOWN) & state_to_set >> 2, test_resource1, aos_resource_entity_root_get());
	printf("set state\n");

	number = DoSub(first, number);
	printf("did first\n");
	if(number == 0) // error
		return;
	number = DoSub(second, number);
	printf("did second\n");
}

