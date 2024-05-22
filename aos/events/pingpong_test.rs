#[cfg(test)]
mod tests {
    use std::path::Path;
    use std::{cell::Cell, time::Duration};

    use aos::configuration::read_config_from;
    use aos::events::event_loop_runtime::{EventLoopRuntimeHolder, Watcher};
    use aos::events::simulated_event_loop::{SimulatedEventLoopFactory, SimulatedEventLoopHolder};
    use aos::testing::init::test_init;
    use aos::testing::path::artifact_path;
    use ping_lib::PingTask;
    use ping_rust_fbs::aos::examples as ping;
    use pong_rust_fbs::aos::examples as pong;

    // We use this trait to simplify leaking memory. For now, the event loop only allows
    // data with a `'static` lifetime. Until that restriction is lifted, we may leak
    // some memory in tests.
    trait Leak: Sized {
        fn leak(self) -> &'static mut Self {
            Box::leak(Box::new(self))
        }
    }

    impl<T> Leak for T {}

    #[allow(unused)]
    struct PingPongTest {
        ping_event_loop: EventLoopRuntimeHolder<SimulatedEventLoopHolder>,
        pong_event_loop: EventLoopRuntimeHolder<SimulatedEventLoopHolder>,
        event_loop_factory: SimulatedEventLoopFactory<'static>,
    }

    impl PingPongTest {
        pub fn init() -> PingPongTest {
            test_init();
            let config =
                read_config_from(&artifact_path(Path::new("aos/events/pingpong_config.json")))
                    .unwrap()
                    .leak();
            let mut event_loop_factory = SimulatedEventLoopFactory::new(config);

            let ping_event_loop = event_loop_factory.make_runtime("ping", None, |runtime| {
                let ping = PingTask::new();
                runtime.spawn(async move { ping.tasks(runtime, 10000).await });
            });

            let pong_event_loop = event_loop_factory.make_runtime("pong", None, |runtime| {
                runtime.spawn(async move { pong_lib::pong(runtime).await })
            });
            PingPongTest {
                event_loop_factory,
                ping_event_loop,
                pong_event_loop,
            }
        }

        pub fn event_loop(&mut self) -> &mut SimulatedEventLoopFactory<'static> {
            &mut self.event_loop_factory
        }
    }

    #[test]
    fn starts() {
        let mut pingpong = PingPongTest::init();
        pingpong.event_loop().run_for(Duration::from_secs(10));
    }

    #[test]
    fn always_replies() {
        let mut pingpong = PingPongTest::init();

        // For now, the simulated event loop requires all references in the tasks
        // to be `'static`, so we leak them for now until the restriction is lifted.
        let ping_count: &Cell<i32> = Cell::new(1).leak();
        let pong_count: &Cell<i32> = Cell::new(1).leak();

        let _test_runtime = pingpong.event_loop().make_runtime("test", None, |runtime| {
            let count_pings = async move {
                let mut ping_watcher: Watcher<ping::Ping> = runtime.make_watcher("/test").unwrap();
                loop {
                    let ping = ping_watcher.next().await;
                    assert_eq!(ping.message().unwrap().value(), ping_count.get());
                    ping_count.set(ping_count.get() + 1);
                }
            };
            let count_pongs = async move {
                let mut pong_watcher: Watcher<pong::Pong> = runtime.make_watcher("/test").unwrap();
                loop {
                    let pong = pong_watcher.next().await;
                    assert_eq!(pong.message().unwrap().value(), pong_count.get());
                    pong_count.set(pong_count.get() + 1);
                }
            };

            runtime.spawn(async move {
                futures::join!(count_pings, count_pongs);
                unreachable!();
            });
        });

        pingpong.event_loop().run_for(Duration::from_secs(10));

        // We run at t=0 and t=10 seconds, which means we run 1 extra time (Note that we started
        // the count at 1, not 0).
        assert_eq!(ping_count.get(), 1002);
        assert_eq!(pong_count.get(), 1002);
    }
}

// TODO(adam.snaider): Remove once we don't leak.
#[no_mangle]
extern "C" fn __asan_default_options() -> *const u8 {
    "detect_leaks=0\0".as_ptr()
}
