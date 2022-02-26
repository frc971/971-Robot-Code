import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time
from typing import Any, Dict, List
import unittest

def write_json(content: Dict[str, Any]):
    """Writes a JSON file with the specified dict content."""
    json_path = Path(os.environ["TEST_TMPDIR"]) / "test.json"
    with open(json_path, "w") as file:
        file.write(json.dumps(content))
    return json_path

def run_debug_cli(args: List[str]):
    run_result = subprocess.run(
        ["scouting/webserver/requests/debug/cli/cli_/cli"] + args,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return (
        run_result.returncode,
        run_result.stdout.decode("utf-8"),
        run_result.stderr.decode("utf-8"),
    )

class TestDebugCli(unittest.TestCase):

    def setUp(self):
        self.webserver = subprocess.Popen(["scouting/webserver/webserver_/webserver"])

        # Wait for the server to respond to requests.
        while True:
            try:
                connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                connection.connect(("localhost", 8080))
                connection.close()
                break
            except ConnectionRefusedError:
                connection.close()
                time.sleep(0.01)

    def tearDown(self):
        self.webserver.terminate()
        self.webserver.wait()

    def test_submit_data_scouting(self):
        json_path = write_json({
            "team": 971,
            "match": 42,
            "upper_goal_hits": 3,
        })
        exit_code, _stdout, stderr = run_debug_cli(["-submitDataScouting", json_path])

        # The SubmitDataScouting message isn't handled yet.
        self.assertEqual(exit_code, 1)
        self.assertIn("/requests/submit/data_scouting returned 501 Not Implemented", stderr)

if __name__ == "__main__":
    unittest.main()
