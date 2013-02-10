extern "C" {
#include "binheap.h"
}

#include <gtest/gtest.h>

class BinHeapTest : public testing::Test{
	protected:
		static const int TEST_ELEMENTS = 57;
		PriorityQueue queue;
		virtual void SetUp(){
			queue = new HeapStruct();
			queue->Elements = new uint8_t[TEST_ELEMENTS];
			Initialize(TEST_ELEMENTS, queue);
		}
		virtual void TearDown(){
			delete[] queue->Elements;
			delete queue;
		}
};

std::ostream& operator<< (std::ostream& o, uint8_t c){
    return o<<(int)c;
}

testing::AssertionResult Contains(PriorityQueue queue, const uint8_t expected[], size_t length){
	for(size_t i = 0; i < length; ++i){
		//printf("expected[%d]=%d\n", i, expected[i]);
		if(DeleteMin(queue) != expected[i]){
			return testing::AssertionFailure() << "queue[" << i << "] != " << expected[i];
		}
	}
	if(!IsEmpty(queue))
		return testing::AssertionFailure() << "queue is longer than " << length;
	return ::testing::AssertionSuccess();
}

TEST_F(BinHeapTest, SingleElement){
	Insert(87, queue);
	EXPECT_EQ(87, DeleteMin(queue));
	EXPECT_TRUE(IsEmpty(queue));
}
TEST_F(BinHeapTest, MultipleElements){
	Insert(54, queue);
	Insert(1, queue);
	Insert(0, queue);
	Insert(255, queue);
	Insert(123, queue);
	uint8_t expected[] = {0, 1, 54, 123, 255};
	EXPECT_TRUE(Contains(queue, expected, sizeof(expected)));
}
TEST_F(BinHeapTest, Removals){
	Insert(54, queue);
	Insert(1, queue);
	Insert(0, queue);
	Insert(255, queue);
	Insert(123, queue);
	Remove(255, queue);
	Remove(0, queue);
	Insert(222, queue);
	Insert(67, queue);
	uint8_t expected[] = {1, 54, 67, 123, 222};
	EXPECT_TRUE(Contains(queue, expected, sizeof(expected)));
}

