use std::{ffi::CString, sync::Once};

autocxx::include_cpp! (
#include "aos/init.h"

safety!(unsafe)

generate!("aos::InitFromRust")
);

/// Initializes things for a test.
///
/// TODO(Brian): Should we provide a proc macro attribute that handles calling this?
///
/// # Panics
///
/// Panics if non-test initialization has already been performed.
pub fn test_init() {
    init();
    // TODO(Brian): Do we want any of the other stuff that `:gtest_main` has?
    // TODO(Brian): Call `aos::SetShmBase` like `:gtest_main` does.
}

pub fn init() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        let argv0 = std::env::args().next().expect("must have argv[0]");
        let argv0 = CString::new(argv0).expect("argv[0] may not have NUL");
        // SAFETY: argv0 is a NUL-terminated string.
        unsafe { ffi::aos::InitFromRust(argv0.as_ptr()) };
    });
}
