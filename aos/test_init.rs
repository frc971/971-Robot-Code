use aos_init::init;

autocxx::include_cpp! (
#include "aos/testing/tmpdir.h"

safety!(unsafe)

generate!("aos::testing::SetTestShmBase")
);

// TODO(Brian): Should we provide a proc macro attribute that handles calling this?
/// Initializes things for a test.
///
/// # Panics
///
/// Panics if non-test initialization has already been performed.
pub fn test_init() {
    init();
    ffi::aos::testing::SetTestShmBase();
    // TODO(Brian): Do we want any of the other stuff that `:gtest_main` has?
}
