# TODO(phil): Rewrite this in Go.

import json
import os
import re
from pathlib import Path
import shutil
import socket
import subprocess
import textwrap
import time
from typing import Any, Dict, List
import unittest

def write_json_request(content: Dict[str, Any]):
    """Writes a JSON file with the specified dict content."""
    json_path = Path(os.environ["TEST_TMPDIR"]) / "test.json"
    json_path.write_text(json.dumps(content))
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


class TestDebugCli(unittest.TestCase):

    def setUp(self):
        tmpdir = Path(os.environ["TEST_TMPDIR"]) / "temp"
        try:
            shutil.rmtree(tmpdir)
        except FileNotFoundError:
            pass
        os.mkdir(tmpdir)

        # Copy the test data into place so that the final API call can be
        # emulated.
        self.set_up_tba_api_dir(tmpdir, year=2016, event_code="nytr")
        self.set_up_tba_api_dir(tmpdir, year=2020, event_code="fake")

        # Create a fake TBA server to serve the static match list.
        self.fake_tba_api = subprocess.Popen(
            ["python3", "-m", "http.server", "7000"],
            cwd=tmpdir,
        )

        # Configure the scouting webserver to scrape data from our fake TBA
        # server.
        scouting_config = tmpdir / "scouting_config.json"
        scouting_config.write_text(json.dumps({
            "api_key": "dummy_key_that_is_not_actually_used_in_this_test",
            "base_url": "http://localhost:7000",
        }))

        # Run the scouting webserver.
        self.webserver = subprocess.Popen([
            "scouting/webserver/webserver_/webserver",
            "-port=8080",
            "-database=%s/database.db" % tmpdir,
            "-tba_config=%s/scouting_config.json" % tmpdir,
        ])

        # Wait for the servers to be reachable.
        wait_for_server(7000)
        wait_for_server(8080)

    def tearDown(self):
        self.fake_tba_api.terminate()
        self.webserver.terminate()
        self.fake_tba_api.wait()
        self.webserver.wait()

    def set_up_tba_api_dir(self, tmpdir, year, event_code):
        tba_api_dir = tmpdir / "api" / "v3" / "event" / f"{year}{event_code}"
        os.makedirs(tba_api_dir)
        (tba_api_dir / "matches").write_text(
            Path(f"scouting/scraping/test_data/{year}_{event_code}.json").read_text()
        )

    def refresh_match_list(self, year=2016, event_code="nytr"):
        """Triggers the webserver to fetch the match list."""
        json_path = write_json_request({
            "year": year,
            "event_code": event_code,
        })
        exit_code, stdout, stderr = run_debug_cli(["-refreshMatchList", json_path])
        self.assertEqual(exit_code, 0, stderr)
        self.assertIn("(refresh_match_list_response.RefreshMatchListResponseT)", stdout)

    def test_submit_and_request_data_scouting(self):
        self.refresh_match_list(year=2020, event_code="fake")

        # First submit some data to be added to the database.
        json_path = write_json_request({
            "team": 100,
            "match": 1,
            "missed_shots_auto": 10,
            "upper_goal_auto": 11,
            "lower_goal_auto": 12,
            "missed_shots_tele": 13,
            "upper_goal_tele": 14,
            "lower_goal_tele": 15,
            "defense_rating": 3,
            "climbing": 1,
        })
        exit_code, _, stderr = run_debug_cli(["-submitDataScouting", json_path])
        self.assertEqual(exit_code, 0, stderr)

        # Now request the data back with zero indentation. That let's us
        # validate the data easily.
        json_path = write_json_request({})
        exit_code, stdout, stderr = run_debug_cli(["-requestDataScouting", json_path, "-indent="])

        self.assertEqual(exit_code, 0, stderr)
        self.assertIn(textwrap.dedent("""\
            {
            Team: (int32) 100,
            Match: (int32) 1,
            MissedShotsAuto: (int32) 10,
            UpperGoalAuto: (int32) 11,
            LowerGoalAuto: (int32) 12,
            MissedShotsTele: (int32) 13,
            UpperGoalTele: (int32) 14,
            LowerGoalTele: (int32) 15,
            DefenseRating: (int32) 3,
            Climbing: (int32) 1
            }"""), stdout)

    def test_request_all_matches(self):
        self.refresh_match_list()

        # RequestAllMatches has no fields.
        json_path = write_json_request({})
        exit_code, stdout, stderr = run_debug_cli(["-requestAllMatches", json_path])

        self.assertEqual(exit_code, 0, stderr)
        self.assertIn("MatchList: ([]*request_all_matches_response.MatchT) (len=90 cap=90) {", stdout)
        self.assertEqual(stdout.count("MatchNumber:"), 90)

    def test_request_matches_for_team(self):
        self.refresh_match_list()

        json_path = write_json_request({
            "team": 4856,
        })
        exit_code, stdout, stderr = run_debug_cli(["-requestMatchesForTeam", json_path])

        # Team 4856 has 12 matches.
        self.assertEqual(exit_code, 0, stderr)
        self.assertIn("MatchList: ([]*request_matches_for_team_response.MatchT) (len=12 cap=12) {", stdout)
        self.assertEqual(stdout.count("MatchNumber:"), 12)
        self.assertEqual(len(re.findall(r": \(int32\) 4856[,\n]", stdout)), 12)

if __name__ == "__main__":
    unittest.main()
