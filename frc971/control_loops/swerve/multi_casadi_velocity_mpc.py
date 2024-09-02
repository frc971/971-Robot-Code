#!/usr/bin/env python3
from absl import app
from frc971.control_loops.swerve import dynamics
from absl import flags
from matplotlib import pylab
import matplotlib
import sys, os, pickle
from multiprocessing.pool import ThreadPool
import numpy
import pathlib, collections
import subprocess
import itertools
import threading

matplotlib.use("GTK3Agg")

FLAGS = flags.FLAGS
flags.DEFINE_string('outdir', '/tmp/swerve', "Directory to write results to.")

workerid_lock = threading.Lock()
workerid_global = 0
workerid = threading.local()


def set_workerid():
    global workerid_global
    with workerid_lock:
        workerid.v = workerid_global
        workerid_global += 1
    print(f'Starting worker {workerid.v}')


Object = lambda **kwargs: type("Object", (), kwargs)


def solve_mpc(velocity):
    filename = f'vx{velocity[0]}vy{velocity[1]}omega{velocity[2]}'
    if FLAGS.outdir:
        subdir = pathlib.Path(FLAGS.outdir) / filename
    else:
        subdir = pathlib.Path(filename)
    subdir.mkdir(parents=True, exist_ok=True)

    subprocess.check_call(args=[
        sys.executable,
        "frc971/control_loops/swerve/casadi_velocity_mpc",
        f"--vx={velocity[0]}",
        f"--vy={velocity[1]}",
        f"--omega={velocity[2]}",
        f"--outputdir={subdir.resolve()}",
        "--pickle",
    ])

    with open(subdir / 't.pkl', 'rb') as f:
        t = pickle.load(f)

    with open(subdir / 'X_plot.pkl', 'rb') as f:
        X_plot = pickle.load(f)

    with open(subdir / 'U_plot.pkl', 'rb') as f:
        U_plot = pickle.load(f)

    return Object(t=t, X_plot=X_plot, U_plot=U_plot)


def main(argv):
    # Load a simple problem first so we compile with less system load.  This
    # makes it faster on a processor with frequency boosting.
    subprocess.check_call(args=[
        sys.executable,
        "frc971/control_loops/swerve/casadi_velocity_mpc",
        "--compileonly",
    ])

    # Try a bunch of goals now
    vxy = numpy.array(
        numpy.meshgrid(numpy.linspace(-1, 1, 9),
                       numpy.linspace(-1, 1, 9))).T.reshape(-1, 2)

    velocity = numpy.hstack((vxy, numpy.zeros((vxy.shape[0], 1))))

    with ThreadPool(initializer=set_workerid) as pool:
        results = pool.starmap(solve_mpc, zip(velocity, ))

    fig0, axs0 = pylab.subplots(2)
    for r in results:
        axs0[0].plot(r.X_plot[dynamics.STATE_VX, :],
                     r.X_plot[dynamics.STATE_VY, :],
                     label='trajectory')
        axs0[1].plot(r.t, r.U_plot[0, :], label="Is0")
        axs0[1].plot(r.t, r.U_plot[1, :], label="Id0")

    axs0[0].legend()
    axs0[1].legend()
    pylab.show()


if __name__ == '__main__':
    app.run(main)
