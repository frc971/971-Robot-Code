#include "aos/common/type_traits.h"

#include "gtest/gtest.h"

namespace aos {
namespace testing {

class BadVirtualFunction {
  virtual void Test() {}
};
TEST(TypeTraitsTest, VirtualFunction) {
  EXPECT_FALSE(shm_ok<BadVirtualFunction>::value);
}
class BadPureVirtualFunction {
  virtual void Test() = 0;
};
TEST(TypeTraitsTest, PureVirtualFunction) {
  EXPECT_FALSE(shm_ok<BadPureVirtualFunction>::value);
}
class BadInheritedVirtual : public BadVirtualFunction {};
TEST(TypeTraitsTest, InheritedVirtualFunction) {
  EXPECT_FALSE(shm_ok<BadInheritedVirtual>::value);
}
class BadVirtualDestructor {
  virtual ~BadVirtualDestructor();
};
TEST(TypeTraitsTest, VirtualDestructor) {
  EXPECT_FALSE(shm_ok<BadVirtualDestructor>::value);
}
class Boring {};
class BadVirtualBase : public virtual Boring {};
TEST(TypeTraitsTest, VirtualBase) {
  EXPECT_FALSE(shm_ok<BadVirtualBase>::value);
}

class GoodSimple {
 public:
  int test1, test2;
  double test3;
};
// Make sure that it lets simple classes through and that the compiler isn't
// completely nuts.
TEST(TypeTraitsTest, Basic) {
  EXPECT_TRUE(shm_ok<GoodSimple>::value);
  GoodSimple test{5, 4, 34.2};
  EXPECT_EQ(5, test.test1);
  EXPECT_EQ(4, test.test2);
  EXPECT_EQ(34.2, test.test3);
  memset(&test, 0, sizeof(test));
  EXPECT_EQ(0, test.test1);
  EXPECT_EQ(0, test.test2);
  EXPECT_EQ(0, test.test3);
}

class GoodWithConstructor {
 public:
  int val_;
  GoodWithConstructor(int val) : val_(val) {}
};
// Make sure that it lets classes with constructors through.
TEST(TypeTraitsTest, GoodWithConstructor) {
  EXPECT_TRUE(shm_ok<GoodWithConstructor>::value);
  GoodWithConstructor test(971);
  EXPECT_EQ(971, test.val_);
}

class GoodPublicPrivateFunction {
 public:
  int32_t a_;
  void set_a(int32_t a) { a_ = a; }
  int32_t b() { return b_; }
  void set_b(int32_t b) { b_ = b; }
 private:
  int32_t b_;
};
// Make sure that member functions still work.
TEST(TypeTraitsTest, Function) {
  EXPECT_TRUE(shm_ok<GoodPublicPrivateFunction>::value);
  EXPECT_EQ(static_cast<unsigned>(8), sizeof(GoodPublicPrivateFunction)) <<
      "The compiler did something weird, but it might not be a problem.";
  GoodPublicPrivateFunction test;
  test.a_ = 5;
  test.set_b(971);
  EXPECT_EQ(5, test.a_);
  EXPECT_EQ(971, test.b());
  test.set_a(74);
  EXPECT_EQ(74, test.a_);
  memset(&test, 0, sizeof(test));
  EXPECT_EQ(0, test.a_);
  EXPECT_EQ(0, test.b());
  test.set_a(123);
  test.set_b(254);
  EXPECT_EQ(123, test.a_);
  EXPECT_EQ(254, test.b());
}

}  // namespace testing
}  // namespace aos
