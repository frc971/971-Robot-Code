#include "aos/common/network/team_number.h"

#include "gtest/gtest.h"

#include "aos/common/macros.h"

namespace aos {
namespace network {
namespace internal {
namespace testing {

TEST(TeamNumberTest, Parse2015TeamNumber) {
  uint16_t team_number;
  EXPECT_EQ(0, ParseTeamNumber("roboRIO-971", &team_number));
  EXPECT_EQ(971u, team_number);

  EXPECT_EQ(0, ParseTeamNumber("roboRIO-8971", &team_number));
  EXPECT_EQ(8971u, team_number);
}

TEST(TeamNumberTest, Parse2016TeamNumber) {
  uint16_t team_number;
  EXPECT_EQ(0, ParseTeamNumber("roboRIO-971-FRC", &team_number));
  EXPECT_EQ(971u, team_number);

  EXPECT_EQ(0, ParseTeamNumber("roboRIO-8971-FRC", &team_number));
  EXPECT_EQ(8971u, team_number);
}

}  // namespace testing
}  // namespace internal
}  // namespace network
}  // namespace aos
