//! These test helpers have to live in a separate file because autocxx only generates one set of
//! outputs per file, and that needs to be the non-`#[cfg(test)]` stuff.

use aos_events_event_loop_runtime::{EventLoop, EventLoopRuntime, RawFetcher};
use ping_rust_fbs::aos::examples::root_as_ping;
use pong_rust_fbs::aos::examples::PongBuilder;

mod tests {
    use super::*;

    use std::cell::RefCell;

    #[derive(Debug, Default)]
    struct GlobalState {
        creation_count: u32,
        drop_count: u32,
        on_run_count: u32,
        before_count: u32,
        watcher_count: u32,
        after_count: u32,
    }

    thread_local!(static GLOBAL_STATE: RefCell<GlobalState> = Default::default());

    fn completed_test_count() -> u32 {
        GLOBAL_STATE.with(|g| {
            let g = &mut *g.borrow_mut();
            let count = g.creation_count;
            assert_eq!(count, g.on_run_count);
            assert_eq!(count, g.before_count);
            assert_eq!(count, g.watcher_count);
            assert_eq!(count, g.after_count);
            assert_eq!(count, g.drop_count);
            count
        })
    }

    pub struct TestApplication<'event_loop> {
        _runtime: EventLoopRuntime<'event_loop>,
        raw_ping_fetcher: RawFetcher,
    }

    impl<'event_loop> TestApplication<'event_loop> {
        fn new(mut runtime: EventLoopRuntime<'event_loop>) -> Self {
            let ping_channel = runtime
                .get_raw_channel("/test", "aos.examples.Ping", "test")
                .expect("Should have Ping channel");
            let mut raw_ping_watcher = runtime.make_raw_watcher(ping_channel);
            let mut raw_pong_sender = runtime.make_raw_sender(
                runtime
                    .get_raw_channel("/test", "aos.examples.Pong", "test")
                    .expect("Should have Pong channel"),
            );
            runtime.spawn(async move {
                // TODO(Brian): Wait for OnRun here.
                GLOBAL_STATE.with(|g| {
                    let g = &mut *g.borrow_mut();
                    assert_eq!(g.creation_count, g.drop_count + 1);
                    assert_eq!(g.drop_count, g.on_run_count);
                    assert_eq!(g.drop_count, g.before_count);
                    assert_eq!(g.drop_count, g.watcher_count);
                    assert_eq!(g.drop_count, g.after_count);
                    g.on_run_count += 1;
                });
                loop {
                    let context = raw_ping_watcher.next().await;
                    assert!(!context.monotonic_event_time().is_min_time());
                    assert!(!context.data().is_none());
                    GLOBAL_STATE.with(|g| {
                        let g = &mut *g.borrow_mut();
                        assert_eq!(g.creation_count, g.drop_count + 1);
                        assert_eq!(g.creation_count, g.on_run_count);
                        assert_eq!(g.creation_count, g.before_count);
                        assert_eq!(g.drop_count, g.watcher_count);
                        assert_eq!(g.drop_count, g.after_count);
                        g.watcher_count += 1;
                    });
                    let ping = root_as_ping(context.data().expect("should have the data"))
                        .expect("Ping should be valid");

                    let mut builder = raw_pong_sender.make_builder();
                    let mut pong = PongBuilder::new(builder.fbb());
                    pong.add_value(ping.value());
                    let pong = pong.finish();
                    // SAFETY: We're sending the correct type here.
                    unsafe { builder.send(pong) }.expect("send should succeed");
                }
            });
            let raw_ping_fetcher = runtime.make_raw_fetcher(ping_channel);
            Self {
                _runtime: runtime,
                raw_ping_fetcher,
            }
        }

        fn before_sending(&mut self) {
            GLOBAL_STATE.with(|g| {
                let g = &mut *g.borrow_mut();
                assert_eq!(g.creation_count, g.drop_count + 1);
                assert_eq!(g.creation_count, g.on_run_count);
                assert_eq!(g.drop_count, g.before_count);
                assert_eq!(g.drop_count, g.watcher_count);
                assert_eq!(g.drop_count, g.after_count);
                g.before_count += 1;
            });
            assert!(
                !self.raw_ping_fetcher.fetch(),
                "should not have message yet"
            );
            assert!(
                !self.raw_ping_fetcher.fetch_next(),
                "should not have message yet"
            );
            let context = self.raw_ping_fetcher.context();
            assert!(context.monotonic_event_time().is_min_time());
            assert!(context.data().is_none());
        }

        fn after_sending(&mut self) {
            GLOBAL_STATE.with(|g| {
                let g = &mut *g.borrow_mut();
                assert_eq!(g.creation_count, g.drop_count + 1);
                assert_eq!(g.creation_count, g.on_run_count);
                assert_eq!(g.creation_count, g.before_count);
                assert_eq!(g.creation_count, g.watcher_count);
                assert_eq!(g.drop_count, g.after_count);
                g.after_count += 1;
            });
            assert!(self.raw_ping_fetcher.fetch(), "should have message now");
            let context = self.raw_ping_fetcher.context();
            assert!(!context.monotonic_event_time().is_min_time());
        }
    }

    impl Drop for TestApplication<'_> {
        fn drop(&mut self) {
            GLOBAL_STATE.with(|g| {
                let g = &mut *g.borrow_mut();
                assert_eq!(g.creation_count, g.drop_count + 1);
                assert_eq!(g.creation_count, g.on_run_count);
                assert_eq!(g.creation_count, g.before_count);
                assert_eq!(g.creation_count, g.watcher_count);
                assert_eq!(g.creation_count, g.after_count);
                g.drop_count += 1;
            });
        }
    }

    unsafe fn make_test_application(event_loop: *mut EventLoop) -> Box<TestApplication<'static>> {
        GLOBAL_STATE.with(|g| {
            let g = &mut *g.borrow_mut();
            g.creation_count += 1;
        });
        Box::new(TestApplication::new(EventLoopRuntime::new(event_loop)))
    }

    #[cxx::bridge(namespace = "aos::events::testing")]
    mod ffi_bridge {
        extern "Rust" {
            type TestApplication<'a>;

            unsafe fn make_test_application(
                event_loop: *mut EventLoop,
            ) -> Box<TestApplication<'static>>;

            fn before_sending(&mut self);
            fn after_sending(&mut self);

            fn completed_test_count() -> u32;
        }

        unsafe extern "C++" {
            include!("aos/events/event_loop.h");
            #[namespace = "aos"]
            type EventLoop = crate::EventLoop;
        }
    }
}
