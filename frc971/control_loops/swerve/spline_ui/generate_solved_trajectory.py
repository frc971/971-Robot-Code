#!/usr/bin/python3
import json

import sys
import gflags
import glog

import trajectory_solver

FLAGS = gflags.FLAGS


def main(argv):
    print(argv)

    # Write the generated jsons out to a file.
    if len(argv) != 3:
        glog.fatal(
            'Expected .json file names for input path and output trajectory.')

    with open(argv[1], 'r') as input_path_file:
        path_data = json.load(input_path_file)

    discretized_trajectory, timestamped_actions = trajectory_solver.solve_and_discretize(
        path_data)

    #turn discretized_trajectory into something our parser can read ):
    discretized_trajectory = [{
        "time": p["time"],
        "position": {
            "x": p["position"][0],
            "y": p["position"][1],
            "theta": p["position"][2]
        },
        "velocity": {
            "x": p["velocity"][0],
            "y": p["velocity"][1],
            "theta": p["velocity"][2]
        },
        "acceleration": {
            "x": p["acceleration"][0],
            "y": p["acceleration"][1],
            "theta": p["acceleration"][2]
        }
    } for p in discretized_trajectory]

    with open(argv[2], "w") as output_file:
        json.dump(
            {
                "discretized_trajectory": discretized_trajectory,
                "timestamped_actions": timestamped_actions
            }, output_file)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
