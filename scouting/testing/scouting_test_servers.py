"""This library is here to run the various servers involved in the scouting app.

The servers are:
 - The fake TBA server
 - The actual web server
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import List

def wait_for_server(port: int):
    """Waits for the server at the specified port to respond to TCP connections."""
    while True:
        try:
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.connect(("localhost", port))
            connection.close()
            break
        except ConnectionRefusedError:
            connection.close()
            time.sleep(0.01)

def create_tba_config(tmpdir: Path) -> Path:
    # Configure the scouting webserver to scrape data from our fake TBA
    # server.
    config = tmpdir / "scouting_config.json"
    config.write_text(json.dumps({
        "api_key": "dummy_key_that_is_not_actually_used_in_this_test",
        "base_url": "http://localhost:7000",
    }))
    return config

def set_up_tba_api_dir(tmpdir: Path, year: int, event_code: str):
    tba_api_dir = tmpdir / "api" / "v3" / "event" / f"{year}{event_code}"
    tba_api_dir.mkdir(parents=True, exist_ok=True)
    (tba_api_dir / "matches").write_text(
        Path(f"scouting/scraping/test_data/{year}_{event_code}.json").read_text()
    )

class Runner:
    """Helps manage the services we need for testing the scouting app."""

    def start(self, port: int):
        """Starts the services needed for testing the scouting app."""
        self.tmpdir = Path(os.environ["TEST_TMPDIR"]) / "servers"
        self.tmpdir.mkdir(exist_ok=True)

        db_path = self.tmpdir / "scouting.db"
        tba_config = create_tba_config(self.tmpdir)

        self.webserver = subprocess.Popen([
            "scouting/scouting",
            f"--port={port}",
            f"--database={db_path}",
            f"--tba_config={tba_config}",
        ])

        # Create a fake TBA server to serve the static match list.
        set_up_tba_api_dir(self.tmpdir, year=2016, event_code="nytr")
        set_up_tba_api_dir(self.tmpdir, year=2020, event_code="fake")
        self.fake_tba_api = subprocess.Popen(
            ["python3", "-m", "http.server", "7000"],
            cwd=self.tmpdir,
        )

        # Wait for the TBA server and the scouting webserver to start up.
        wait_for_server(7000)
        wait_for_server(port)

    def stop(self):
        """Stops the services needed for testing the scouting app."""
        servers = (self.webserver, self.fake_tba_api)
        for server in servers:
            server.terminate()
        for server in servers:
            server.wait()

        try:
            shutil.rmtree(self.tmpdir)
        except FileNotFoundError:
            pass


def main(argv: List[str]):
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, help="The port for the actual web server.")
    args = parser.parse_args(argv[1:])

    runner = Runner()
    runner.start(args.port)

    # Wait until we're asked to shut down via CTRL-C or SIGTERM.
    signal.pause()

    runner.stop()


if __name__ == "__main__":
    sys.exit(main(sys.argv))