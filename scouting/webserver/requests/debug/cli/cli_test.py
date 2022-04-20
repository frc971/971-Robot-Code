# TODO(phil): Rewrite this in Go.

import json
import os
import re
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import textwrap
import time
from typing import Any, Dict, List
import unittest

import scouting.testing.scouting_test_servers


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


class TestDebugCli(unittest.TestCase):

    def setUp(self):
        self.servers = scouting.testing.scouting_test_servers.Runner()
        self.servers.start(8080)

    def tearDown(self):
        self.servers.stop()

    def refresh_match_list(self, year=2016, event_code="nytr"):
        """Triggers the webserver to fetch the match list."""
        json_path = write_json_request({
            "year": year,
            "event_code": event_code,
        })
        exit_code, stdout, stderr = run_debug_cli(["-refreshMatchList", json_path])
        self.assertEqual(exit_code, 0, f"{year}{event_code}: {stderr}")
        self.assertIn("(refresh_match_list_response.RefreshMatchListResponseT)", stdout)

    def test_submit_and_request_data_scouting(self):
        self.refresh_match_list(year=2020, event_code="fake")

        # First submit some data to be added to the database.
        json_path = write_json_request({
            "team": 100,
            "match": 1,
            "set_number": 2,
            "comp_level": "quals",
            "starting_quadrant": 3,
            "auto_ball_1": True,
            "auto_ball_2": False,
            "auto_ball_3": False,
            "auto_ball_4": False,
            "auto_ball_5": True,
            "missed_shots_auto": 10,
            "upper_goal_auto": 11,
            "lower_goal_auto": 12,
            "missed_shots_tele": 13,
            "upper_goal_tele": 14,
            "lower_goal_tele": 15,
            "defense_rating": 3,
            "defense_received_rating": 4,
            "climb_level": "Medium",
            "comment": "A very inspiring and useful comment",
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
            CollectedBy: (string) (len=9) "debug_cli",
            AutoBall1: (bool) true,
            AutoBall2: (bool) false,
            AutoBall3: (bool) false,
            AutoBall4: (bool) false,
            AutoBall5: (bool) true,
            StartingQuadrant: (int32) 3,
            ClimbLevel: (request_data_scouting_response.ClimbLevel) Medium,
            DefenseReceivedRating: (int32) 4,
            Comment: (string) (len=35) "A very inspiring and useful comment",
            SetNumber: (int32) 2,
            CompLevel: (string) (len=5) "quals"
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

    def test_request_all_matches(self):
        """Makes sure that we can import the match list multiple times without problems."""
        request_all_matches_outputs = []
        for _ in range(2):
            self.refresh_match_list()

            # RequestAllMatches has no fields.
            json_path = write_json_request({})
            exit_code, stdout, stderr = run_debug_cli(["-requestAllMatches", json_path])

            self.assertEqual(exit_code, 0, stderr)
            request_all_matches_outputs.append(stdout)

        self.maxDiff = None
        self.assertEqual(request_all_matches_outputs[0], request_all_matches_outputs[1])

if __name__ == "__main__":
    unittest.main()
