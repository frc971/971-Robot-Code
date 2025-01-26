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

from rules_python.python.runfiles import runfiles

import scouting.testing.scouting_test_servers

RUNFILES = runfiles.Create()

# This regex finds the number of matches that the web server has imported. This
# is intended to parse the output of the debug cli's `-requestAllMatches`
# option.
MATCH_LIST_LENGTH_EXTRACTION_RE = re.compile(
    r"MatchList: \(\[\]\*request_all_matches_response.MatchT\) \(len=(\d+) cap=.*\) \{"
)


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


def find_num_matches_for_event(year, event_code):
    with open(
            RUNFILES.Rlocation(
                f"org_frc971/scouting/scraping/test_data/{year}_{event_code}.json"
            ), "r") as file:
        raw_match_list = json.load(file)
    return len(raw_match_list)


class TestDebugCli(unittest.TestCase):

    def setUp(self):
        self.servers = scouting.testing.scouting_test_servers.Runner()

    def tearDown(self):
        self.servers.stop()

    def start_servers(self, year=2016, event_code="nytr"):
        self.servers.start(8080, year=year, event_code=event_code)

        expected_num_matches = find_num_matches_for_event(year, event_code)
        json_path = write_json_request({})

        # Wait until the match list is imported. This is done automatically
        # when the web server starts up.
        sys.stderr.write("Waiting for match list to be imported.\n")
        while True:
            exit_code, stdout, stderr = run_debug_cli(
                ["-requestAllMatches", json_path])
            self.assertEqual(exit_code, 0, stderr)

            match = MATCH_LIST_LENGTH_EXTRACTION_RE.search(stdout)
            if match:
                num_matches_imported = int(match.group(1))

                if num_matches_imported == expected_num_matches:
                    break
                else:
                    sys.stderr.write(
                        f"Waiting until {expected_num_matches} are imported. "
                        f"Currently at {num_matches_imported}.\n")
            else:
                sys.stderr.write(
                    "Failed to parse requestAllMatches for number of "
                    f"matches: {stdout}\n")

            time.sleep(0.25)

    def test_request_current_scouting(self):
        self.start_servers(year=2020, event_code="fake")

        # First submit some data to be added to the database.
        json_path = write_json_request({"team_number": "971"})
        exit_code, _, stderr = run_debug_cli(
            ["-requestCurrentScouting", json_path])
        self.assertEqual(exit_code, 0, stderr)

    def test_submit_and_request_notes(self):
        self.start_servers(year=2020, event_code="fake")

        # First submit some data to be added to the database.
        json_path = write_json_request({
            "team": "100",
            "notes": "A very inspiring and useful comment",
            "good_driving": True,
            "bad_driving": False,
            "solid_placing": False,
            "sketchy_placing": True,
            "no_show": False,
            "match_number": 3,
            "set_number": 1,
            "comp_level": "qm",
        })
        exit_code, _, stderr = run_debug_cli(["-submitNotes", json_path])
        self.assertEqual(exit_code, 0, stderr)

        # Now request the data back with zero indentation. That let's us
        # validate the data easily.
        json_path = write_json_request({})
        exit_code, stdout, stderr = run_debug_cli(
            ["-requestAllNotes", json_path, "-indent="])

        self.assertEqual(exit_code, 0, stderr)
        self.assertIn(
            textwrap.dedent("""\
            {
            Team: (string) (len=3) "100",
            Notes: (string) (len=35) "A very inspiring and useful comment",
            GoodDriving: (bool) true,
            BadDriving: (bool) false,
            SolidPlacing: (bool) false,
            SketchyPlacing: (bool) true,
            GoodDefense: (bool) false,
            BadDefense: (bool) false,
            EasilyDefended: (bool) false,
            NoShow: (bool) false,
            MatchNumber: (int32) 3,
            SetNumber: (int32) 1,
            CompLevel: (string) (len=2) "qm"
            }"""), stdout)

    def test_submit_and_request_notes_2025(self):
        self.start_servers(year=2020, event_code="fake")

        # First submit some data to be added to the database.
        json_path = write_json_request({
            "comp_code": "2020mttd",
            "team_number": "100",
            "match_number": 3,
            "set_number": 1,
            "comp_level": "qm",
            "notes": "A very inspiring and useful comment",
            "good_driving": True,
            "bad_driving": False,
            "coral_ground_intake": False,
            "coral_hp_intake": True,
            "algae_ground_intake": True,
            "solid_algae_shooting": False,
            "sketchy_algae_shooting": False,
            "solid_coral_shooting": False,
            "sketchy_coral_shooting": False,
            "shuffle_coral": False,
            "penalties": False,
            "good_defense": True,
            "bad_defense": True,
            "easily_defended": True,
            "no_show": False,
        })
        exit_code, _, stderr = run_debug_cli(["-submitNotes2025", json_path])
        self.assertEqual(exit_code, 0, stderr)

        # Now request the data back with zero indentation. That let's us
        # validate the data easily.
        json_path = write_json_request({
            "comp_code": "2020mttd",
        })
        exit_code, stdout, stderr = run_debug_cli(
            ["-requestAllNotes2025", json_path, "-indent="])

        self.assertEqual(exit_code, 0, stderr)
        self.assertIn(
            textwrap.dedent("""\
            {
            CompCode: (string) (len=8) "2020mttd",
            TeamNumber: (string) (len=3) "100",
            MatchNumber: (int32) 3,
            SetNumber: (int32) 1,
            CompLevel: (string) (len=2) "qm",
            Notes: (string) (len=35) "A very inspiring and useful comment",
            GoodDriving: (bool) true,
            BadDriving: (bool) false,
            CoralGroundIntake: (bool) false,
            CoralHpIntake: (bool) true,
            AlgaeGroundIntake: (bool) true,
            SolidAlgaeShooting: (bool) false,
            SketchyAlgaeShooting: (bool) false,
            SolidCoralShooting: (bool) false,
            SketchyCoralShooting: (bool) false,
            ShuffleCoral: (bool) false,
            Penalties: (bool) false,
            GoodDefense: (bool) true,
            BadDefense: (bool) true,
            EasilyDefended: (bool) true,
            NoShow: (bool) false
            }"""), stdout)

    def test_submit_and_request_driver_ranking(self):
        self.start_servers(year=2020, event_code="fake")

        # First submit some data to be added to the database.
        json_path = write_json_request({
            "match_number": 100,
            "rank1": "101",
            "rank2": "202",
            "rank3": "303",
        })
        exit_code, _, stderr = run_debug_cli(
            ["-submitDriverRanking", json_path])
        self.assertEqual(exit_code, 0, stderr)

        # Now request the data back with zero indentation. That let's us
        # validate the data easily.
        json_path = write_json_request({})
        exit_code, stdout, stderr = run_debug_cli(
            ["-requestAllDriverRankings", json_path, "-indent="])

        self.assertEqual(exit_code, 0, stderr)
        self.assertIn(
            textwrap.dedent("""\
            {
            MatchNumber: (int32) 100,
            Rank1: (string) (len=3) "101",
            Rank2: (string) (len=3) "202",
            Rank3: (string) (len=3) "303"
            }"""), stdout)

    def test_request_all_matches(self):
        self.start_servers()

        # RequestAllMatches has no fields.
        json_path = write_json_request({})
        exit_code, stdout, stderr = run_debug_cli(
            ["-requestAllMatches", json_path])

        self.assertEqual(exit_code, 0, stderr)
        self.assertIn(
            "MatchList: ([]*request_all_matches_response.MatchT) (len=90 cap=90) {",
            stdout)
        self.assertEqual(stdout.count("MatchNumber:"), 90)


if __name__ == "__main__":
    unittest.main()
