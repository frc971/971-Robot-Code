#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"
#include "y2024/constants/constants_list_generated.h"

namespace y2024 {
namespace constants {
namespace testing {
class ConstantsValidatorTest : public ::testing::Test {};

TEST_F(ConstantsValidatorTest, CheckConstants) {
  CHECK_NOTNULL(aos::JsonFileToFlatbuffer<y2024::ConstantsList>(
                    "y2024/constants/constants.json")
                    .message()
                    .constants());
}

}  // namespace testing
}  // namespace constants
}  // namespace y2024
