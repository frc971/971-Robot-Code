#include "aos/network/team_number.h"

#include "gtest/gtest.h"

namespace aos::network::testing {

using team_number_internal::ParsePiOrOrinTeamNumber;
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

TEST(TeamNumberTest, ParsePiOrOrinTeamNumber) {
  EXPECT_EQ(971u, *ParsePiOrOrinTeamNumber("pi-971-1"));
  EXPECT_EQ(8971u, *ParsePiOrOrinTeamNumber("pi-8971-22"));
  EXPECT_EQ(8971u, *ParsePiOrOrinTeamNumber("pi-8971-"));

  EXPECT_EQ(971u, *ParsePiOrOrinTeamNumber("orin-971-1"));
  EXPECT_EQ(8971u, *ParsePiOrOrinTeamNumber("orin-8971-22"));
  EXPECT_EQ(8971u, *ParsePiOrOrinTeamNumber("orin-8971-"));

  EXPECT_FALSE(ParseRoborioTeamNumber("pi"));
  EXPECT_FALSE(ParseRoborioTeamNumber("pi-"));
  EXPECT_FALSE(ParseRoborioTeamNumber("pi-971"));
  EXPECT_FALSE(ParseRoborioTeamNumber("pi-971a-1"));
  EXPECT_FALSE(ParseRoborioTeamNumber("orin-971-1"));

  EXPECT_EQ(1u, *ParsePiOrOrinNumber("pi-971-1"));
  EXPECT_EQ(22u, *ParsePiOrOrinNumber("pi-8971-22"));
  EXPECT_EQ(1u, *ParsePiOrOrinNumber("orin-971-1"));
  EXPECT_EQ(22u, *ParsePiOrOrinNumber("orin-8971-22"));

  EXPECT_FALSE(ParsePiOrOrinNumber("pi-8971-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("pi"));
  EXPECT_FALSE(ParsePiOrOrinNumber("pi-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("pi-971"));

  EXPECT_FALSE(ParsePiOrOrinNumber("orin-8971-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("orin"));
  EXPECT_FALSE(ParsePiOrOrinNumber("orin-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("orin-971"));
}

}  // namespace aos::network::testing
