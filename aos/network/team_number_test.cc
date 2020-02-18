#include "aos/network/team_number.h"

#include "gtest/gtest.h"

namespace aos {
namespace network {
namespace testing {

using team_number_internal::ParseRoborioTeamNumber;

TEST(TeamNumberTest, Parse2015TeamNumber) {
  EXPECT_EQ(971u, *ParseRoborioTeamNumber("roboRIO-971"));

  EXPECT_EQ(8971u, ParseRoborioTeamNumber("roboRIO-8971"));

  EXPECT_FALSE(ParseRoborioTeamNumber("abc"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-8abc"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-"));
}

TEST(TeamNumberTest, Parse2016TeamNumber) {
  EXPECT_EQ(971u, *ParseRoborioTeamNumber("roboRIO-971-FRC"));

  EXPECT_EQ(8971u, *ParseRoborioTeamNumber("roboRIO-8971-FRC"));

  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-8abc-FRC"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-8971-FRC2"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-8971-2FRC"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO--FRC"));
}

}  // namespace testing
}  // namespace network
}  // namespace aos
