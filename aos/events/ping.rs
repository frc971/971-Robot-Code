use aos_configuration as config;
use aos_events_event_loop_runtime::{EventLoopRuntime, Sender, Watcher};
use aos_events_shm_event_loop::ShmEventLoop;
use core::cell::Cell;
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
        runtime.spawn(ping.tasks(runtime));
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
    pub async fn tasks(&self, event_loop: &EventLoopRuntime<'_>) -> Never {
        futures::join!(self.ping(event_loop), self.handle_pong(event_loop));
        unreachable!("Let's hope `never_type` gets stabilized soon :)");
    }

    async fn ping(&self, event_loop: &EventLoopRuntime<'_>) -> Never {
        // The sender is used to send messages back to the pong channel.
        let mut ping_sender: Sender<ping::Ping> = event_loop.make_sender("/test").unwrap();
        let mut interval = event_loop.add_interval(Duration::from_secs(1));

        event_loop.on_run().await;
        loop {
            interval.tick().await;
            self.counter.set(self.counter.get() + 1);
            let mut builder = ping_sender.make_builder();
            let mut ping = ping::PingBuilder::new(builder.fbb());
            let iter = self.counter.get();
            ping.add_value(iter);
            ping.add_send_time(event_loop.monotonic_now().into());
            let ping = ping.finish();
            builder.send(ping).expect("Can't send ping");
        }
    }

    async fn handle_pong(&self, event_loop: &EventLoopRuntime<'_>) -> Never {
        // The watcher gives us incoming ping messages.
        let mut pong_watcher: Watcher<pong::Pong> = event_loop.make_watcher("/test").unwrap();

        event_loop.on_run().await;
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
