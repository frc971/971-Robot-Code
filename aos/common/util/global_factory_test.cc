#include "aos/common/util/global_factory.h"
#include "gtest/gtest.h"

namespace aos {

namespace test_a {
class BaseType {
 public:
  virtual ~BaseType() {}

  virtual std::pair<int, int> Get() = 0;
};

SETUP_FACTORY(BaseType, int, int);

class BaseTypeNoArgs {
 public:
  virtual ~BaseTypeNoArgs() {}

  virtual int Get() = 0;
};

SETUP_FACTORY(BaseTypeNoArgs);

}  // namespace test_a

namespace test_b {

class SubType : public test_a::BaseType {
 public:
  SubType(int t1, int t2) : value_(t1, t2) {}
  std::pair<int, int> Get() override { return value_; }

 private:
  std::pair<int, int> value_;
};

REGISTER_SUBCLASS(test_a::BaseType, SubType);

}  // namespace test_b

namespace {

class SubType1 : public test_a::BaseTypeNoArgs {
 public:
  int Get() override { return 1; }
};

class SubType2 : public test_a::BaseTypeNoArgs {
 public:
  int Get() override { return 2; }
};
REGISTER_SUBCLASS(test_a::BaseTypeNoArgs, SubType1);
REGISTER_SUBCLASS(test_a::BaseTypeNoArgs, SubType2);

TEST(GlobalFactoryTest, CheckFactory) {
  auto val = test_a::BaseTypeGlobalFactory::Get("SubType")(2, 7)->Get();
  EXPECT_EQ(val.first, 2);
  EXPECT_EQ(val.second, 7);
}
TEST(GlobalFactoryTest, CheckFactoryNoArgs) {
  EXPECT_EQ(1, test_a::BaseTypeNoArgsGlobalFactory::Get("SubType1")()->Get());
  EXPECT_EQ(2, test_a::BaseTypeNoArgsGlobalFactory::Get("SubType2")()->Get());
}

}  // namespace
}  // namespace aos
