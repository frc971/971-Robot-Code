use aos_configuration as config;
use aos_events_event_loop_runtime::{EventLoopRuntime, Sender, Watcher};
use aos_events_shm_event_loop::ShmEventLoop;
use futures::never::Never;
use std::path::Path;

use ping_rust_fbs::aos::examples as ping;
use pong_rust_fbs::aos::examples as pong;

fn main() {
    aos_init::init();
    let config = config::read_config_from(Path::new("pingpong_config.json")).unwrap();
    ShmEventLoop::new(&config).run_with(|runtime| {
        let task = pong(runtime);
        runtime.spawn(task);
    });
}

/// Responds to ping messages with an equivalent pong.
async fn pong(event_loop: &EventLoopRuntime<'_>) -> Never {
    // The watcher gives us incoming ping messages.
    let mut ping_watcher: Watcher<ping::Ping> = event_loop.make_watcher("/test").unwrap();

    // The sender is used to send messages back to the pong channel.
    let mut pong_sender: Sender<pong::Pong> = event_loop.make_sender("/test").unwrap();

    event_loop.on_run().await;
    loop {
        let ping = dbg!(ping_watcher.next().await);

        let mut builder = pong_sender.make_builder();
        let mut pong = pong::PongBuilder::new(builder.fbb());
        pong.add_value(ping.message().unwrap().value());
        pong.add_initial_send_time(event_loop.monotonic_now().into());
        let pong = pong.finish();
        builder.send(pong).expect("Can't send pong reponse");
    }
}
