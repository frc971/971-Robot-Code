use aos_configuration as config;
use aos_events_shm_event_loop::ShmEventLoop;
use aos_init::WithCppFlags;
use clap::Parser;
use std::path::Path;

use pong_lib::pong;

/// Pong portion of a ping/pong system.
#[derive(Parser, Debug)]
#[command(name = "pong")]
struct App {}

fn main() {
    let _app = App::parse_with_cpp_flags();
    aos_init::init();
    let config = config::read_config_from(Path::new("pingpong_config.json")).unwrap();
    ShmEventLoop::new(&config).run_with(|runtime| {
        let task = pong(*runtime);
        runtime.set_realtime_priority(5);
        runtime.spawn(task);
    });
}
