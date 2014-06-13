#include "aos/common/util/options.h"

#include "gtest/gtest.h"

namespace aos {
namespace testing {

class OptionsTest : public ::testing::Test {
 public:
  static constexpr Options<OptionsTest>::Option kOne{1}, kTwo{2}, kThree{4},
      kFour{8};
};

constexpr Options<OptionsTest>::Option OptionsTest::kOne, OptionsTest::kTwo,
    OptionsTest::kThree, OptionsTest::kFour;

TEST_F(OptionsTest, Basic) {
  const Options<OptionsTest> one_three = kOne | kThree;
  EXPECT_TRUE(one_three & kOne);
  EXPECT_FALSE(one_three & kTwo);
  EXPECT_TRUE(one_three & kThree);
}

TEST_F(OptionsTest, NoOthersSet) {
  const Options<OptionsTest> one_three = kOne | kThree;
  EXPECT_TRUE(one_three.NoOthersSet(one_three));
  EXPECT_TRUE(one_three.NoOthersSet(kOne | kTwo | kThree));
  EXPECT_TRUE(one_three.NoOthersSet(kOne | kThree | kFour));
  EXPECT_TRUE(one_three.NoOthersSet(kOne | kTwo | kThree | kFour));
  EXPECT_FALSE(one_three.NoOthersSet(kOne));
  EXPECT_FALSE(one_three.NoOthersSet(kThree));
  EXPECT_FALSE(one_three.NoOthersSet(kTwo | kFour));
}

TEST_F(OptionsTest, ExactlyOneSet) {
  const Options<OptionsTest> one_three = kOne | kThree;
  EXPECT_TRUE(one_three.ExactlyOneSet(kOne | kTwo));
  EXPECT_FALSE(one_three.ExactlyOneSet(one_three));
  EXPECT_TRUE(one_three.ExactlyOneSet(kTwo | kThree | kFour));
  EXPECT_FALSE(one_three.ExactlyOneSet(kOne | kTwo | kThree | kFour));
}

TEST_F(OptionsTest, AllSet) {
  const Options<OptionsTest> one_three = kOne | kThree;
  EXPECT_TRUE(one_three.AllSet(one_three));
  EXPECT_TRUE(one_three.AllSet(kOne));
  EXPECT_FALSE(one_three.AllSet(kTwo));
  EXPECT_TRUE(one_three.AllSet(kThree));
  EXPECT_FALSE(one_three.AllSet(kFour));
  EXPECT_FALSE(one_three.AllSet(kOne | kTwo | kFour));
  EXPECT_FALSE(one_three.AllSet(kTwo | kThree | kFour));
  EXPECT_FALSE(one_three.AllSet(kOne | kTwo | kThree | kFour));
}

}  // namespace testing
}  // namespace aos
