#!/usr/bin/python3

import numpy
import math
import scipy.integrate

from matplotlib import pylab
import sys
import gflags
import glog

from frc971.control_loops.swerve.dynamics import swerve_physics

FLAGS = gflags.FLAGS


def u_func(X):
    result = numpy.zeros([8, 1])
    # result[1, 0] = 80.0
    # result[3, 0] = 80.0
    # result[5, 0] = -80.0
    # result[7, 0] = -80.0
    return result


def main(argv):
    x_initial = numpy.zeros([25, 1])
    # x_initial[0] = -math.pi / 2.0
    x_initial[3] = 1.0 / (2.0 * 0.0254)
    # x_initial[4] = math.pi / 2.0
    x_initial[7] = 1.0 / (2.0 * 0.0254)
    # x_initial[8] = -math.pi / 2.0
    x_initial[11] = 1.0 / (2.0 * 0.0254)
    # x_initial[12] = math.pi / 2.0
    x_initial[15] = 1.0 / (2.0 * 0.0254)
    x_initial[19] = 2.0
    result = scipy.integrate.solve_ivp(swerve_physics, (0, 2.0),
                                       x_initial.reshape(25, ),
                                       max_step=0.01,
                                       args=(u_func, ))

    cm = pylab.get_cmap('gist_rainbow')
    fig = pylab.figure()
    ax = fig.add_subplot(111)
    ax.set_prop_cycle(color=[cm(1. * i / 25) for i in range(25)])
    ax.plot(result.t, result.y[0, :], label="thetas0", linewidth=7.0)
    ax.plot(result.t, result.y[1, :], label="thetad0", linewidth=7.0)
    ax.plot(result.t, result.y[2, :], label="omegas0", linewidth=7.0)
    ax.plot(result.t, result.y[3, :], label="omegad0", linewidth=7.0)
    ax.plot(result.t, result.y[4, :], label="thetas1", linewidth=7.0)
    ax.plot(result.t, result.y[5, :], label="thetad1", linewidth=7.0)
    ax.plot(result.t, result.y[6, :], label="omegas1", linewidth=7.0)
    ax.plot(result.t, result.y[7, :], label="omegad1", linewidth=7.0)
    ax.plot(result.t, result.y[8, :], label="thetas2", linewidth=7.0)
    ax.plot(result.t, result.y[9, :], label="thetad2", linewidth=7.0)
    ax.plot(result.t, result.y[10, :], label="omegas2", linewidth=7.0)
    ax.plot(result.t, result.y[11, :], label="omegad2", linewidth=7.0)
    ax.plot(result.t, result.y[12, :], label="thetas3", linewidth=7.0)
    ax.plot(result.t, result.y[13, :], label="thetad3", linewidth=7.0)
    ax.plot(result.t, result.y[14, :], label="omegas3", linewidth=7.0)
    ax.plot(result.t, result.y[15, :], label="omegad3", linewidth=7.0)
    ax.plot(result.t, result.y[16, :], label="x", linewidth=7.0)
    ax.plot(result.t, result.y[17, :], label="y", linewidth=7.0)
    ax.plot(result.t, result.y[18, :], label="theta", linewidth=7.0)
    ax.plot(result.t, result.y[19, :], label="vx", linewidth=7.0)
    ax.plot(result.t, result.y[20, :], label="vy", linewidth=7.0)
    ax.plot(result.t, result.y[21, :], label="omega", linewidth=7.0)
    ax.plot(result.t, result.y[22, :], label="Fx", linewidth=7.0)
    ax.plot(result.t, result.y[23, :], label="Fy", linewidth=7.0)
    ax.plot(result.t, result.y[24, :], label="Moment", linewidth=7.0)
    numpy.set_printoptions(threshold=numpy.inf)
    print(result.t)
    print(result.y)
    pylab.legend()
    pylab.show()

    return 0


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
