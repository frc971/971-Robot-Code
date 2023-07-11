use std::{marker::PhantomData, pin::Pin, ptr};

use autocxx::WithinBox;
use cxx::UniquePtr;

pub use aos_configuration::{Channel, Configuration, ConfigurationExt, Node};
use aos_configuration_fbs::aos::Configuration as RustConfiguration;
pub use aos_events_event_loop_runtime::{CppExitHandle, EventLoop, ExitHandle};
use aos_events_event_loop_runtime::{EventLoopHolder, EventLoopRuntime, EventLoopRuntimeHolder};
use aos_flatbuffers::{transmute_table_to, Flatbuffer};

autocxx::include_cpp! (
#include "aos/events/simulated_event_loop.h"
#include "aos/events/simulated_event_loop_for_rust.h"

safety!(unsafe)

generate!("aos::SimulatedEventLoopFactoryForRust")

extern_cpp_type!("aos::ExitHandle", crate::CppExitHandle)
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
    pub fn new(config: &'config impl Flatbuffer<RustConfiguration<'static>>) -> Self {
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
        // * The return value manages its lifetime via `unique_ptr`.
        //
        // Note that dropping `self` before the return value will abort from C++, but this is still
        // sound.
        unsafe {
            self.as_mut()
                .MakeEventLoop(name, node.map_or(ptr::null(), |p| p))
        }
    }

    /// Creates an [`EventLoopRuntime`] wrapper which also owns its underlying [`EventLoop`].
    ///
    /// All setup must be performed with `fun`, which is called before this function returns. `fun`
    /// may create further objects to use in async functions via [`EventLoop.spawn`] etc, but it is
    /// the only place to set things up before the EventLoop is run.
    ///
    /// Note that dropping the return value will drop this EventLoop.
    pub fn make_runtime<F>(
        &mut self,
        name: &str,
        node: Option<&Node>,
        fun: F,
    ) -> EventLoopRuntimeHolder<SimulatedEventLoopHolder>
    where
        F: for<'event_loop> FnOnce(&mut EventLoopRuntime<'event_loop>),
    {
        let event_loop = self.make_event_loop(name, node);
        // SAFETY: We just created this EventLoop, so we are the exclusive owner of it.
        let holder = unsafe { SimulatedEventLoopHolder::new(event_loop) };
        EventLoopRuntimeHolder::new(holder, fun)
    }

    pub fn make_exit_handle(&mut self) -> ExitHandle {
        self.as_mut().MakeExitHandle().into()
    }

    pub fn run(&mut self) {
        self.as_mut().Run();
    }

    // TODO(Brian): Expose OnStartup. Just take a callback for creating things, and rely on
    // dropping the created objects instead of OnShutdown.
    // pub fn spawn_on_startup(&mut self, spawner: impl FnMut());
}

pub struct SimulatedEventLoopHolder(UniquePtr<EventLoop>);

impl SimulatedEventLoopHolder {
    /// SAFETY: `event_loop` must be the exclusive owner of the underlying EventLoop.
    pub unsafe fn new(event_loop: UniquePtr<EventLoop>) -> Self {
        Self(event_loop)
    }
}

// SAFETY: The UniquePtr functions we're using here mirror most of the EventLoopHolder requirements
// exactly. Safety requirements on [`SimulatedEventLoopHolder.new`] take care of the rest.
unsafe impl EventLoopHolder for SimulatedEventLoopHolder {
    fn into_raw(self) -> *mut ffi::aos::EventLoop {
        self.0.into_raw()
    }

    unsafe fn from_raw(raw: *mut ffi::aos::EventLoop) -> Self {
        Self(UniquePtr::from_raw(raw))
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

            let exit_handle = event_loop_factory.make_exit_handle();
            let _runtime1 = {
                let pi1_ref = &pi1;
                event_loop_factory.make_runtime("runtime1", pi1, move |runtime1| {
                    assert!(pi1_ref.unwrap().has_name());
                    let channel = runtime1
                        .get_raw_channel("/test", "aos.examples.Ping")
                        .unwrap();
                    let mut watcher = runtime1.make_raw_watcher(channel);
                    runtime1.spawn(async move {
                        watcher.next().await;
                        GLOBAL_STATE.with(|g| {
                            let g = &mut *g.borrow_mut();
                            g.watcher_count = g.watcher_count + 1;
                        });
                        exit_handle.exit().await
                    });
                })
            };

            let _runtime2 = {
                let pi1_ref = &pi1;
                event_loop_factory.make_runtime("runtime2", pi1, |runtime2| {
                    assert!(pi1_ref.unwrap().has_name());
                    let channel = runtime2
                        .get_raw_channel("/test", "aos.examples.Ping")
                        .unwrap();
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
                })
            };

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
