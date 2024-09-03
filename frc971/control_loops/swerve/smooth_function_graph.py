#!/usr/bin/python3

import numpy
import scipy
import matplotlib.pyplot as plt
import matplotlib
from scipy.special import logsumexp

x = numpy.linspace(-10, 10, 1000)
y0 = numpy.zeros((1000, 1))
y1 = numpy.zeros((1000, 1))

# add more detail near 0
X = numpy.sort(
    numpy.hstack(
        [numpy.arange(-1, 1, 0.005),
         numpy.arange(-0.005, 0.005, 0.0001)]))
Y = numpy.sort(
    numpy.hstack(
        [numpy.arange(-1, 1, 0.005),
         numpy.arange(-0.005, 0.005, 0.0001)]))
X, Y = numpy.meshgrid(X, Y)


def single_slip_force(vy, vx):
    return numpy.arctan2(vy, numpy.abs(vx))


def single_new_slip_force(vy, vx):
    loggain = 1 / 0.05
    return numpy.arctan2(vy,
                         logsumexp([1.0, numpy.abs(vx) * loggain]) / loggain)


def single_new_new_slip_force(vy, vx):
    loggain = 1 / 0.05
    loggain2 = 1 / 0.05
    return numpy.arctan2(
        vy,
        logsumexp(
            [1.0, vx * (1.0 - 2.0 /
                        (1 + numpy.exp(loggain2 * vx))) * loggain]) / loggain)


velocity = 0.1

acc_val = single_new_new_slip_force(velocity, velocity)
expt_val = single_new_slip_force(velocity, velocity)

print("Percent Error: ", (acc_val - expt_val) / expt_val * 100)

slip_force = scipy.vectorize(single_slip_force)
new_slip_force = scipy.vectorize(single_new_slip_force)
new_new_slip_force = scipy.vectorize(single_new_new_slip_force)

Y0 = slip_force(Y, X)
Y1 = new_slip_force(Y, X)
Y2 = new_new_slip_force(Y, X)
Y3 = Y2 - Y1

matplotlib.rcParams['figure.figsize'] = (15, 15)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(X,
                       Y,
                       Y0,
                       cmap=matplotlib.cm.coolwarm,
                       linewidth=0,
                       antialiased=False)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('atan2(y, x)')
fig.suptitle("Atan2")

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(X,
                       Y,
                       Y1,
                       cmap=matplotlib.cm.coolwarm,
                       linewidth=0,
                       antialiased=False)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('atan2(y, x)')
fig.suptitle("Softened y atan2")

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(X,
                       Y,
                       Y2,
                       cmap=matplotlib.cm.coolwarm,
                       linewidth=0,
                       antialiased=False)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('atan2(y, x)')
fig.suptitle("Softened x and y atan2")

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(X,
                       Y,
                       Y3,
                       cmap=matplotlib.cm.coolwarm,
                       linewidth=0,
                       antialiased=False)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Error')
fig.suptitle("Error between Soft_atan2 and the new one")

plt.show()
