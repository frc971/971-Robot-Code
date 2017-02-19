#include "aos/vision/blob/transpose.h"
#include "aos/vision/blob/test_utils.h"

#include <algorithm>
#include <string>
#include "gtest/gtest.h"

namespace aos {
namespace vision {

TEST(TransposeTest, Tranpspose) {
  RangeImage img = LoadFromTestData(20, R"(
    -----------
    -----  ----
   ------------
   -------------
   ------------
    ----------
   ------------
     ---------
)");

  auto b = Transpose(img);
  auto c = Transpose(b);
  EXPECT_EQ(ShortDebugPrint({img}), ShortDebugPrint({c}));
}

}  // namespace vision
}  // namespace aos
