// Reexport all of the [`aos`] crate
pub use aos::*;

/// Utilities for testing an AOS application.
pub mod testing {
    pub use aos_test_init as init;
}
