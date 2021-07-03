import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
import numpy as np
from points import Points
from libspline import Spline, DistanceSpline, Trajectory

from matplotlib.backends.backend_gtk3agg import (FigureCanvasGTK3Agg as
                                                 FigureCanvas)
from matplotlib.figure import Figure


class Graph(Gtk.Bin):
    def __init__(self):
        super(Graph, self).__init__()
        fig = Figure(figsize=(5, 4), dpi=100)
        self.axis = fig.add_subplot(111)
        canvas = FigureCanvas(fig)  # a Gtk.DrawingArea
        canvas.set_vexpand(True)
        canvas.set_size_request(800, 250)
        self.add(canvas)

    def recalculate_graph(self, points):
        if not points.getLibsplines(): return

        # set the size of a timestep
        dt = 0.00505

        # call C++ wrappers to calculate the trajectory
        distanceSpline = DistanceSpline(points.getLibsplines())
        traj = Trajectory(distanceSpline)
        points.addConstraintsToTrajectory(traj)
        traj.Plan()
        XVA = traj.GetPlanXVA(dt)

        # extract values to be graphed
        total_steps_taken = XVA.shape[1]
        total_time = dt * total_steps_taken
        time = np.arange(total_time, step=dt)
        position, velocity, acceleration = XVA
        left_voltage, right_voltage = zip(*(traj.Voltage(x) for x in position))

        # update graph
        self.axis.clear()
        self.axis.plot(time, velocity)
        self.axis.plot(time, acceleration)
        self.axis.plot(time, left_voltage)
        self.axis.plot(time, right_voltage)
        self.axis.legend(
            ["Velocity", "Acceleration", "Left Voltage", "Right Voltage"])
        self.axis.xaxis.set_label_text("Time (sec)")

        # renumber the x-axis to include the last point,
        # the total time to drive the spline
        self.axis.xaxis.set_ticks(np.linspace(0, total_time, num=8))
