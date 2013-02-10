#include "ResourceActionChild.h"

void ResourceActionChild_t::DoAction(uint8_t actions){
	if(actions & 0x01)
		if(TryRequestResource(test_resource1)){
			PostStatus(false);
			return;
		}
	if(actions & 0x02)
		ReleaseResource(test_resource1);
	PostStatus(true);
}

