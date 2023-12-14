use aos::events::event_loop_runtime::{EventLoopRuntime, Sender, Watcher};
use futures::never::Never;
use std::borrow::Borrow;

use ping_rust_fbs::aos::examples as ping;
use pong_rust_fbs::aos::examples as pong;

/// Responds to ping messages with an equivalent pong.
pub async fn pong(event_loop: EventLoopRuntime<'_>) -> Never {
    // The watcher gives us incoming ping messages.
    let mut ping_watcher: Watcher<ping::Ping> = event_loop.make_watcher("/test").unwrap();

    // The sender is used to send messages back to the pong channel.
    let mut pong_sender: Sender<pong::Pong> = event_loop.make_sender("/test").unwrap();

    let on_run = event_loop.on_run();
    on_run.borrow().await;
    loop {
        let ping = ping_watcher.next().await;
        let ping = ping.message().unwrap();
        log::info!("Got ping: {}", ping.value());

        let mut builder = pong_sender.make_builder();
        let mut pong = pong::PongBuilder::new(builder.fbb());
        pong.add_value(ping.value());
        pong.add_initial_send_time(event_loop.monotonic_now().into());
        let pong = pong.finish();
        builder.send(pong).expect("Can't send pong reponse");
    }
}
