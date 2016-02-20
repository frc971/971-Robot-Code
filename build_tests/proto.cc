#include "gtest/gtest.h"

#include "build_tests/proto.pb.h"

TEST(ProtoBuildTest, Serialize) {
  ::frc971::TestProto test_proto1, test_proto2;
  test_proto1.set_s("Hi!");
  test_proto1.set_i(971);

  ::std::string serialized;
  ASSERT_TRUE(test_proto1.SerializeToString(&serialized));
  ASSERT_TRUE(test_proto2.ParseFromString(serialized));

  EXPECT_EQ("Hi!", test_proto2.s());
  EXPECT_EQ(971, test_proto2.i());
}
