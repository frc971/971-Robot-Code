use aos::configuration;
use aos::events::shm_event_loop::ShmEventLoop;
use aos::init::{DefaultApp, Init};
use std::path::Path;

use pong_lib::pong;

fn main() {
    let _ = DefaultApp::init();
    let config = configuration::read_config_from(Path::new("pingpong_config.json")).unwrap();
    ShmEventLoop::new(&config).run_with(|runtime| {
        let task = pong(*runtime);
        runtime.set_realtime_priority(5);
        runtime.spawn(task);
    });
}
