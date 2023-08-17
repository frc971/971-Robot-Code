use aos_configuration as config;
use aos_events_event_loop_runtime::{EventLoopRuntime, Sender, Watcher};
use aos_events_shm_event_loop::ShmEventLoop;
use core::cell::Cell;
use core::future::Future;
use core::time::Duration;
use futures::never::Never;
use std::path::Path;

use ping_rust_fbs::aos::examples as ping;
use pong_rust_fbs::aos::examples as pong;

fn main() {
    aos_init::init();
    let config = config::read_config_from(Path::new("pingpong_config.json")).unwrap();
    let ping = PingTask::new();
    ShmEventLoop::new(&config).run_with(|runtime| {
        let task = ping.tasks(runtime);
        runtime.spawn(task);
    });
}

#[derive(Debug)]
struct PingTask {
    counter: Cell<i32>,
}

impl PingTask {
    pub fn new() -> Self {
        Self {
            counter: Cell::new(0),
        }
    }

    /// Returns a future with all the tasks for the ping process
    pub fn tasks(&self, event_loop: &mut EventLoopRuntime) -> impl Future<Output = Never> + '_ {
        let ping = self.ping(event_loop);
        let handle_pong = self.handle_pong(event_loop);

        async move {
            futures::join!(ping, handle_pong);
            unreachable!("Let's hope `never_type` gets stabilized soon :)");
        }
    }

    fn ping(&self, event_loop: &mut EventLoopRuntime) -> impl Future<Output = Never> + '_ {
        // The sender is used to send messages back to the pong channel.
        let mut ping_sender: Sender<ping::Ping> = event_loop.make_sender("/test").unwrap();
        let startup = event_loop.on_run();

        let mut interval = event_loop.add_interval(Duration::from_secs(1));

        async move {
            // Wait for startup.
            startup.await;
            loop {
                interval.tick().await;
                self.counter.set(self.counter.get() + 1);
                let mut builder = ping_sender.make_builder();
                let mut ping = ping::PingBuilder::new(builder.fbb());
                let iter = self.counter.get();
                ping.add_value(iter);
                let ping = ping.finish();
                builder.send(ping).expect("Can't send ping");
            }
        }
    }

    fn handle_pong(&self, event_loop: &mut EventLoopRuntime) -> impl Future<Output = Never> + '_ {
        // The watcher gives us incoming ping messages.
        let mut pong_watcher: Watcher<pong::Pong> = event_loop.make_watcher("/test").unwrap();
        let startup = event_loop.on_run();

        async move {
            // Wait for startup.
            startup.await;
            loop {
                let pong = dbg!(pong_watcher.next().await);
                assert_eq!(
                    pong.message().unwrap().value(),
                    self.counter.get(),
                    "Missed a reply"
                );
            }
        }
    }
}
