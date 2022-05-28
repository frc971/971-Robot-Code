import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
import numpy as np
import queue
import threading
import copy
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
        self.canvas = FigureCanvas(fig)  # a Gtk.DrawingArea
        self.canvas.set_vexpand(True)
        self.canvas.set_size_request(800, 250)
        self.add(self.canvas)
        self.queue = queue.Queue(maxsize=1)

        thread = threading.Thread(target=self.worker)
        thread.daemon = True
        thread.start()

    def schedule_recalculate(self, points):
        if not points.getLibsplines() or self.queue.full(): return
        new_copy = copy.deepcopy(points)

        # empty the queue
        try:
            self.queue.get_nowait()
        except queue.Empty:
            pass # was already empty

        # replace with new request
        self.queue.put_nowait(new_copy)

    def worker(self):
        while True:
            self.recalculate_graph(self.queue.get())

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
        if XVA is None: return

        # extract values to be graphed
        total_steps_taken = XVA.shape[1]
        total_time = dt * total_steps_taken
        time = np.linspace(0, total_time, num=total_steps_taken)
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

        # redraw
        self.canvas.draw()
