#include "gtest/gtest.h"

#include "aos/common/logging/logging.h"
#include "aos/protobuf/stack_arena.h"
#include "aos/testing/test_logging.h"

namespace aos {
namespace protobuf {
namespace {

struct TestStruct {
  int a;
  int b;
  int c;
};

class StackProtoArenaTest : public ::testing::Test {};

TEST_F(StackProtoArenaTest, Basic) {
  ::aos::testing::EnableTestLogging();
  StackProtoArena<164> stack_arena;

  TestStruct* msg =
      google::protobuf::Arena::Create<TestStruct>(stack_arena.arena());
  CHECK_NOTNULL(msg);
}

}  // namespace
}  // namespace protobuf
}  // namespace aos
