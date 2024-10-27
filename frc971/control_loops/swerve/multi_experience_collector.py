#!/usr/bin/env python3
from absl import app
from absl import flags
import sys
from multiprocessing.pool import ThreadPool
import pathlib
import subprocess

FLAGS = flags.FLAGS
flags.DEFINE_string('outdir', '/tmp/swerve', "Directory to write results to.")
flags.DEFINE_integer('num_actors', 20, 'Number of actors to run in parallel.')
flags.DEFINE_integer('num_solutions', 100,
                     'Number of random problems to solve.')


def collect_experience(agent_number):
    filename = f'{agent_number}'
    if FLAGS.outdir:
        subdir = pathlib.Path(FLAGS.outdir) / filename
    else:
        subdir = pathlib.Path(filename)
    subdir.mkdir(parents=True, exist_ok=True)

    with open(f'{subdir.resolve()}/log', 'w') as output:
        subprocess.check_call(
            args=[
                sys.executable,
                "frc971/control_loops/swerve/experience_collector",
                f"--seed={agent_number}",
                f"--outputdir={subdir.resolve()}",
                "--quiet",
                f"--num_solutions={FLAGS.num_solutions}",
            ],
            stdout=output,
            stderr=output,
        )


def main(argv):
    # Load a simple problem first so we compile with less system load.  This
    # makes it faster on a processor with frequency boosting.
    subprocess.check_call(args=[
        sys.executable,
        "frc971/control_loops/swerve/experience_collector",
        "--compileonly",
    ])

    # Try a bunch of goals now
    with ThreadPool(FLAGS.num_actors) as pool:
        pool.starmap(collect_experience, zip(range(FLAGS.num_actors), ))


if __name__ == '__main__':
    app.run(main)
