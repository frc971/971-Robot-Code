#include "y2018/control_loops/superstructure/debouncer.h"

#include "gtest/gtest.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace testing {

// Tests that the debouncer behaves as it should. This tests the following:
// - The debouncer changes its internal state after the desired number of
// repeated inputs.
// - The debouncer doesn't change its internal state before the desired number
// of repeated inputs.
TEST(DebouncerTest, Debouncer) {
  Debouncer bouncer(false, 2);

  bouncer.Update(true);
  bouncer.Update(true);
  EXPECT_EQ(true, bouncer.current_state());

  bouncer.Update(false);

  // Only one false, state shouldn't have changed.
  EXPECT_EQ(true, bouncer.current_state());

  bouncer.Update(false);
  // Now there are two falses in a row, the state should've changed.
  EXPECT_EQ(false, bouncer.current_state());
}

// Test that the debouncer will hold its state through a short-lived state
// change.
TEST(DebouncerTest, DebouncerLongSequence) {
  Debouncer bouncer(false, 2);

  bouncer.Update(true);

  // Only one true, should still read false.
  EXPECT_EQ(false, bouncer.current_state());

  bouncer.Update(true);

  // Two trues, should now read true.
  EXPECT_EQ(true, bouncer.current_state());

  bouncer.Update(false);

  // Only one false, should still read true.
  EXPECT_EQ(true, bouncer.current_state());

  bouncer.Update(true);

  EXPECT_EQ(true, bouncer.current_state());
}
}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
