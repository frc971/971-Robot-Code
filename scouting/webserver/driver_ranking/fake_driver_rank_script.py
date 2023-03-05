"""A dummy script that helps validate driver_ranking.go logic.

Since we don't have Julia support, we can't run the real script.
"""

import argparse
import csv
import sys
from pathlib import Path

EXPECTED_INPUT = [
    [
        'Timestamp', 'Scout Name', 'Match Number', 'Alliance', 'Rank 1 (best)',
        'Rank 2', 'Rank 3 (worst)'
    ],
    ["", "", "", "", "1234", "1235", "1236"],
    ["", "", "", "", "971", "972", "973"],
]

OUTPUT = [
    ("1234", "1.5"),
    ("1235", "2.75"),
    ("1236", "4.0"),
    ("971", "5.25"),
    ("972", "6.5"),
    ("973", "7.75"),
]


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    args = parser.parse_args(argv[1:])

    print("Reading input CSV")
    with args.input_csv.open("r") as input_csv:
        reader = csv.reader(input_csv)
        input_data = [row for row in reader]

    if EXPECTED_INPUT != input_data:
        raise ValueError("Input data mismatch. Got: " + str(input_data))

    print("Generating output CSV")
    with args.output_csv.open("w") as output_csv:
        writer = csv.writer(output_csv)
        for row in OUTPUT:
            writer.writerow(row)

    print("Successfully generated fake parsed driver ranking data.")


if __name__ == "__main__":
    sys.exit(main(sys.argv))
