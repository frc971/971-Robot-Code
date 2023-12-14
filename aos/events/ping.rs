use aos::configuration;
use aos::events::shm_event_loop::ShmEventLoop;
use aos::init::Init;
use clap::Parser;
use ping_lib::PingTask;
use std::path::Path;

/// Ping portion of a ping/pong system.
#[derive(Parser, Debug)]
struct App {
    /// Time to sleep between pings.
    #[arg(long, default_value_t = 10000, value_name = "MICROS")]
    sleep: u64,
}

fn main() {
    let app = App::init();
    let config = configuration::read_config_from(Path::new("pingpong_config.json")).unwrap();
    let ping = PingTask::new();
    ShmEventLoop::new(&config).run_with(|runtime| {
        runtime.set_realtime_priority(5);
        runtime.spawn(ping.tasks(*runtime, app.sleep));
    });
}
