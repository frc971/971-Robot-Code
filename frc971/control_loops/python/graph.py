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
        self.callback_id = self.canvas.mpl_connect('motion_notify_event',
                                                   self.on_mouse_move)
        self.add(self.canvas)

        # The current graph data
        self.data = None
        # The size of a timestep
        self.dt = 0.00505
        # The position of the cursor in seconds
        self.cursor = 0

        # Reference to the parent gtk Widget that wants to get redrawn
        # when user moves the cursor
        self.cursor_watcher = None
        self.cursor_line = None

        self.queue = queue.Queue(maxsize=1)
        thread = threading.Thread(target=self.worker)
        thread.daemon = True
        thread.start()

    def find_cursor(self):
        """Gets the cursor position as a distance along the spline"""
        if self.data is None:
            return None
        cursor_index = int(self.cursor / self.dt)
        # use the time to index into the position data
        distance_at_cursor = self.data[0][cursor_index - 1]
        return distance_at_cursor

    def place_cursor(self, distance):
        """Places the cursor at a certain distance along the spline"""
        if self.data is None:
            return
        # convert distance along spline to time along trajectory
        index = np.searchsorted(self.data[0], distance, side='left')
        time = index * self.dt
        self.cursor = time
        self.redraw_cursor()

    def on_mouse_move(self, event):
        """Updates the cursor and all the canvases that watch it on mouse move"""
        if self.data is None:
            return
        total_steps_taken = self.data.shape[1]
        total_time = self.dt * total_steps_taken
        if event.xdata is not None:
            # clip the position if still on the canvas, but off the graph
            self.cursor = np.clip(event.xdata, 0, total_time)

            self.redraw_cursor()

            # tell the field to update too
            if self.cursor_watcher is not None:
                self.cursor_watcher.queue_draw()

    def redraw_cursor(self):
        """Redraws the cursor line"""
        # TODO: This redraws the entire graph and isn't very snappy
        if self.cursor_line: self.cursor_line.remove()
        self.cursor_line = self.axis.axvline(self.cursor)
        self.canvas.draw_idle()

    def schedule_recalculate(self, points):
        """Submits points to be graphed

        Can be superseded by newer points if an old one isn't finished processing.
        """
        if not points.getLibsplines(): return
        new_copy = copy.deepcopy(points)

        # empty the queue
        try:
            self.queue.get_nowait()
        except queue.Empty:
            pass  # was already empty

        # replace with new request
        self.queue.put_nowait(new_copy)

    def worker(self):
        while True:
            self.recalculate_graph(self.queue.get())

    def recalculate_graph(self, points):
        if not points.getLibsplines(): return

        # call C++ wrappers to calculate the trajectory
        distance_spline = DistanceSpline(points.getLibsplines())
        traj = Trajectory(distance_spline)
        points.addConstraintsToTrajectory(traj)
        traj.Plan()
        self.data = traj.GetPlanXVA(self.dt)
        if self.data is None: return

        # extract values to be graphed
        total_steps_taken = self.data.shape[1]
        total_time = self.dt * total_steps_taken
        times = np.linspace(0, total_time, num=total_steps_taken)
        position, velocity, acceleration = self.data
        left_voltage, right_voltage = zip(*(traj.Voltage(x) for x in position))

        # update graph
        self.axis.clear()
        self.axis.plot(times, velocity)
        self.axis.plot(times, acceleration)
        self.axis.plot(times, left_voltage)
        self.axis.plot(times, right_voltage)
        self.axis.legend(
            ["Velocity", "Acceleration", "Left Voltage", "Right Voltage"])
        self.axis.xaxis.set_label_text("Time (sec)")

        # renumber the x-axis to include the last point,
        # the total time to drive the spline
        self.axis.xaxis.set_ticks(np.linspace(0, total_time, num=8))

        # redraw
        self.canvas.draw_idle()
