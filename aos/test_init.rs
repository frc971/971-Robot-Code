use std::sync::Once;

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
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        aos_init::internal::init();
        ffi::aos::testing::SetTestShmBase();
        env_logger::builder().is_test(true).init();
        // TODO(Brian): Do we want any of the other stuff that `:gtest_main` has?
    });
}
