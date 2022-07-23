use std::{
    marker::PhantomData,
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
    pin::Pin,
    ptr,
};

use autocxx::WithinBox;
use cxx::UniquePtr;
use futures::{future::pending, never::Never};

pub use aos_configuration::{Channel, Configuration, ConfigurationExt, Node};
use aos_configuration_fbs::aos::Configuration as RustConfiguration;
pub use aos_events_event_loop_runtime::EventLoop;
use aos_events_event_loop_runtime::EventLoopRuntime;
use aos_flatbuffers::{transmute_table_to, Flatbuffer};

autocxx::include_cpp! (
#include "aos/events/simulated_event_loop.h"
#include "aos/events/simulated_event_loop_for_rust.h"

safety!(unsafe)

generate!("aos::ExitHandle")
generate!("aos::SimulatedEventLoopFactoryForRust")

extern_cpp_type!("aos::Configuration", crate::Configuration)
extern_cpp_type!("aos::Node", crate::Node)
extern_cpp_type!("aos::EventLoop", crate::EventLoop)
);

/// A Rust-owned C++ `SimulatedEventLoopFactory` object.
///
/// Owning one of these via a heap allocation makes things a lot simpler, and all the functions
/// that are called repeatedly are heavyweight enough this is not a performance concern.
///
/// We don't want to own the `SimulatedEventLoopFactory` directly because C++ maintains multiple
/// pointers to it, so we can't create Rust mutable references to it.
pub struct SimulatedEventLoopFactory<'config> {
    // SAFETY: This stores a pointer to the configuration, whose lifetime is `'config`.
    event_loop_factory: Pin<Box<ffi::aos::SimulatedEventLoopFactoryForRust>>,
    // This represents the config pointer C++ is storing.
    _marker: PhantomData<&'config Configuration>,
}

impl<'config> SimulatedEventLoopFactory<'config> {
    pub fn new<'new_config: 'config>(
        config: &'new_config impl Flatbuffer<RustConfiguration<'static>>,
    ) -> Self {
        // SAFETY: `_marker` represents the lifetime of this pointer we're handing off to C++ to
        // store.
        let event_loop_factory = unsafe {
            ffi::aos::SimulatedEventLoopFactoryForRust::new(transmute_table_to::<Configuration>(
                &config.message()._tab,
            ))
        }
        .within_box();
        Self {
            event_loop_factory,
            _marker: PhantomData,
        }
    }

    fn as_mut(&mut self) -> Pin<&mut ffi::aos::SimulatedEventLoopFactoryForRust> {
        self.event_loop_factory.as_mut()
    }

    /// Creates a Rust-owned EventLoop.
    ///
    /// You probably don't want to call this directly if you're creating a Rust application. This
    /// is intended for creating C++ applications. Use [`make_runtime`] instead when creating Rust
    /// applications.
    pub fn make_event_loop(&mut self, name: &str, node: Option<&Node>) -> UniquePtr<EventLoop> {
        // SAFETY:
        // * `self` has a valid C++ object.
        // * C++ doesn't need the lifetimes of `name` or `node` to last any longer than this method
        //   call.
        // * The return value is `'static` because it's wrapped in `unique_ptr`.
        //
        // Note that dropping `self` before the return value will abort from C++, but this is still
        // sound.
        unsafe {
            self.as_mut()
                .MakeEventLoop(name, node.map_or(ptr::null(), |p| p))
        }
    }

    /// Creates an [`EventLoopRuntime`] wrapper which also owns its underlying EventLoop.
    pub fn make_runtime(&mut self, name: &str, node: Option<&Node>) -> SimulatedEventLoopRuntime {
        SimulatedEventLoopRuntime::new(self.make_event_loop(name, node))
    }

    pub fn make_exit_handle(&mut self) -> ExitHandle {
        ExitHandle(self.as_mut().MakeExitHandle())
    }

    pub fn run(&mut self) {
        self.as_mut().Run();
    }

    // TODO(Brian): Expose OnStartup. Just take a callback for creating things, and rely on
    // dropping the created objects instead of OnShutdown.
    // pub fn spawn_on_startup(&mut self, spawner: impl FnMut());
}

// TODO(Brian): Move this and the `generate!` somewhere else once we wrap ShmEventLoop, which also
// uses it.
pub struct ExitHandle(UniquePtr<ffi::aos::ExitHandle>);

impl ExitHandle {
    /// Exits the EventLoops represented by this handle. You probably want to immediately return
    /// from the context this is called in. Awaiting [`exit`] instead of using this function is an
    /// easy way to do that.
    pub fn exit_sync(mut self) {
        self.0.as_mut().unwrap().Exit();
    }

    /// Exits the EventLoops represented by this handle, and never returns. Immediately awaiting
    /// this from a [`EventLoopRuntime::spawn`]ed task is usually what you want, it will ensure
    /// that no more code from that task runs.
    pub async fn exit(self) -> Never {
        self.exit_sync();
        pending().await
    }
}

pub struct SimulatedEventLoopRuntime(ManuallyDrop<EventLoopRuntime<'static>>);

impl SimulatedEventLoopRuntime {
    pub fn new(event_loop: UniquePtr<EventLoop>) -> Self {
        // SAFETY: We own the underlying EventLoop, so `'static` is the correct lifetime. Anything
        // using this `EventLoopRuntime` will need to borrow the object we're returning, which will
        // ensure it stays alive.
        let runtime = unsafe { EventLoopRuntime::<'static>::new(event_loop.into_raw()) };
        Self(ManuallyDrop::new(runtime))
    }
}

impl Drop for SimulatedEventLoopRuntime {
    fn drop(&mut self) {
        let event_loop = self.raw_event_loop();
        // SAFETY: We're not going to touch this field again.
        unsafe { ManuallyDrop::drop(&mut self.0) };
        // SAFETY: `new` created this from `into_raw`. We just dropped the only Rust reference to
        // it.
        unsafe { UniquePtr::from_raw(event_loop) };
    }
}

impl Deref for SimulatedEventLoopRuntime {
    type Target = EventLoopRuntime<'static>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for SimulatedEventLoopRuntime {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::cell::RefCell;

    use futures::future::pending;
    use runfiles::Runfiles;

    use aos_configuration::read_config_from;
    use aos_init::test_init;
    use ping_rust_fbs::aos::examples::PingBuilder;

    // A really basic test of the functionality here.
    #[test]
    fn smoke_test() {
        #[derive(Debug, Default)]
        struct GlobalState {
            watcher_count: u32,
            startup_count: u32,
        }

        thread_local!(static GLOBAL_STATE: RefCell<GlobalState> = Default::default());

        test_init();
        let r = Runfiles::create().unwrap();
        let config = read_config_from(
            &r.rlocation("org_frc971/aos/events/multinode_pingpong_test_combined_config.json"),
        )
        .unwrap();
        let mut event_loop_factory = SimulatedEventLoopFactory::new(&config);
        {
            let pi1 = Some(config.message().get_node("pi1").unwrap());
            let mut runtime1 = event_loop_factory.make_runtime("runtime1", pi1);
            let channel = runtime1
                .configuration()
                .get_channel("/test", "aos.examples.Ping", "test", pi1)
                .unwrap();
            let mut runtime2 = event_loop_factory.make_runtime("runtime2", pi1);

            {
                let mut watcher = runtime1.make_raw_watcher(channel);
                let exit_handle = event_loop_factory.make_exit_handle();
                runtime1.spawn(async move {
                    watcher.next().await;
                    GLOBAL_STATE.with(|g| {
                        let g = &mut *g.borrow_mut();
                        g.watcher_count = g.watcher_count + 1;
                    });
                    exit_handle.exit().await
                });
            }

            {
                let mut sender = runtime2.make_raw_sender(channel);
                runtime2.spawn(async move {
                    GLOBAL_STATE.with(|g| {
                        let g = &mut *g.borrow_mut();
                        g.startup_count = g.startup_count + 1;
                    });

                    let mut builder = sender.make_builder();
                    let ping = PingBuilder::new(builder.fbb()).finish();
                    // SAFETY: We're using the correct message type.
                    unsafe { builder.send(ping) }.expect("send should succeed");
                    pending().await
                });
            }

            GLOBAL_STATE.with(|g| {
                let g = g.borrow();
                assert_eq!(0, g.watcher_count);
                // TODO(Brian): Use an OnRun wrapper to defer setting this until it actually starts,
                // then check it.
                //assert_eq!(0, g.startup_count);
            });
            event_loop_factory.run();
            GLOBAL_STATE.with(|g| {
                let g = g.borrow();
                assert_eq!(1, g.watcher_count);
                assert_eq!(1, g.startup_count);
            });
        }
    }
}
