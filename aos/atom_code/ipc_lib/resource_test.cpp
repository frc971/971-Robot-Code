#include "aos/atom_code/ipc_lib/sharedmem_test_setup.h"
#include "aos/aos_core.h"

#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>

class ResourceTest : public SharedMemTestSetup{
};

TEST_F(ResourceTest, GetResource){
	aos_resource *first = aos_resource_get(1);
	AllSharedMemAllocated();
	aos_resource *second = aos_resource_get(1);
	EXPECT_EQ(first, second);
}
TEST_F(ResourceTest, CheckLocal){
	EXPECT_EQ(1, aos_resource_request(NULL, aos_resource_get(2)));
}

TEST_F(ResourceTest, LocalCreate){
	EXPECT_TRUE(aos_resource_entity_create(56) != NULL);
}
TEST_F(ResourceTest, LocalSetParentSelf){
	aos_resource_entity *local = aos_resource_entity_create(76);
	ASSERT_TRUE(local != NULL);
	EXPECT_EQ(1, aos_resource_entity_set_parent(local, local));
}

