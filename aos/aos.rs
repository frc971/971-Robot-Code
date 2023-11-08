//! All of AOS into a single, easy to use library.

pub use aos_configuration as configuration;
pub use aos_init as init;
pub use aos_uuid as uuid;

pub use aos_flatbuffers as flatbuffers;

/// The essentials for working with the AOS event loop.
pub mod events {
    pub use aos_events_event_loop_runtime as event_loop_runtime;
    pub use aos_events_shm_event_loop as shm_event_loop;
    pub use aos_events_simulated_event_loop as simulated_event_loop;
}
