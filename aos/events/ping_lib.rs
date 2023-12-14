use aos::events::event_loop_runtime::{EventLoopRuntime, Sender, Watcher};
use core::cell::Cell;
use core::time::Duration;
use futures::never::Never;
use std::borrow::Borrow;

use ping_rust_fbs::aos::examples as ping;
use pong_rust_fbs::aos::examples as pong;

#[derive(Debug)]
pub struct PingTask {
    counter: Cell<i32>,
}

impl PingTask {
    pub fn new() -> Self {
        Self {
            counter: Cell::new(0),
        }
    }

    /// Returns a future with all the tasks for the ping process
    #[allow(unreachable_code)]
    pub async fn tasks(&self, event_loop: EventLoopRuntime<'_>, sleep: u64) -> Never {
        futures::join!(self.ping(&event_loop, sleep), self.handle_pong(&event_loop));
        unreachable!("Let's hope `never_type` gets stabilized soon :)");
    }

    pub async fn ping(&self, event_loop: &EventLoopRuntime<'_>, sleep: u64) -> Never {
        // The sender is used to send messages back to the pong channel.
        let mut ping_sender: Sender<ping::Ping> = event_loop.make_sender("/test").unwrap();
        let mut interval = event_loop.add_interval(Duration::from_micros(sleep));

        let on_run = event_loop.on_run();
        on_run.borrow().await;

        loop {
            interval.tick().await;
            self.counter.set(self.counter.get() + 1);
            let mut builder = ping_sender.make_builder();
            let mut ping = ping::PingBuilder::new(builder.fbb());
            let iter = self.counter.get();
            log::trace!("Ping: {iter}");
            ping.add_value(iter);
            ping.add_send_time(event_loop.monotonic_now().into());
            let ping = ping.finish();
            builder.send(ping).expect("Can't send ping");
        }
    }

    pub async fn handle_pong(&self, event_loop: &EventLoopRuntime<'_>) -> Never {
        // The watcher gives us incoming ping messages.
        let mut pong_watcher: Watcher<pong::Pong> = event_loop.make_watcher("/test").unwrap();

        let on_run = event_loop.on_run();
        on_run.borrow().await;
        loop {
            let pong = pong_watcher.next().await;
            let pong = pong.message().unwrap();
            log::trace!("Got pong: {}", pong.value());
            assert_eq!(pong.value(), self.counter.get(), "Missed a reply");
        }
    }
}
