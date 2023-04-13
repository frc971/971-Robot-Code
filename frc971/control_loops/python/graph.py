import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
import numpy as np
import queue
import threading
import copy
from multispline import Multispline
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
        self.mouse_move_callback = self.canvas.mpl_connect(
            'motion_notify_event', self.on_mouse_move)
        self.click_callback = self.canvas.mpl_connect('button_press_event',
                                                      self.on_click)
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
        if self.data[0].size < cursor_index:
            return None
        # use the time to index into the position data
        try:
            distance_at_cursor = self.data[0][cursor_index - 1]
            multispline_index = int(self.data[5][cursor_index - 1])
            return (multispline_index, distance_at_cursor)
        except IndexError:
            return None

    def place_cursor(self, multispline_index, distance):
        """Places the cursor at a certain distance along the spline"""
        if self.data is None:
            return

        # find the section that is the current multispline
        start_of_multispline = np.searchsorted(self.data[5],
                                               multispline_index,
                                               side='left')
        end_of_multispline = np.searchsorted(self.data[5],
                                             multispline_index,
                                             side='right')
        multispline_region = self.data[0][
            start_of_multispline:end_of_multispline]

        # convert distance along this multispline to time along trajectory
        index = np.searchsorted(multispline_region, distance,
                                side='left') + start_of_multispline
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

    def on_click(self, event):
        """Same as on_mouse_move but also selects multisplines"""

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
                self.cursor_watcher.on_graph_clicked()

    def redraw_cursor(self):
        """Redraws the cursor line"""
        # TODO: This redraws the entire graph and isn't very snappy
        if self.cursor_line: self.cursor_line.remove()
        self.cursor_line = self.axis.axvline(self.cursor)
        self.canvas.draw_idle()

    def schedule_recalculate(self, multisplines):
        """Submits points to be graphed

        Can be superseded by newer points if an old one isn't finished processing.
        """

        new_copy = copy.deepcopy(multisplines)

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

    def recalculate_graph(self, multisplines):
        if len(multisplines) == 0: return

        # call C++ wrappers to calculate the trajectory
        full_data = None

        for multispline_index, multispline in enumerate(multisplines):
            multispline.update_lib_spline()
            if len(multispline.getLibsplines()) == 0: continue
            distanceSpline = DistanceSpline(multispline.getLibsplines())
            traj = Trajectory(distanceSpline)
            multispline.addConstraintsToTrajectory(traj)
            traj.Plan()
            XVA = traj.GetPlanXVA(self.dt)
            if XVA is None: continue
            position, _, _ = XVA

            voltages = np.transpose([traj.Voltage(x) for x in position])

            data = np.append(XVA, voltages, axis=0)

            indicies = np.full((1, XVA.shape[1]), multispline_index, dtype=int)
            data = np.append(data, indicies, axis=0)

            if full_data is not None:
                full_data = np.append(full_data, data, axis=1)
            else:
                full_data = data

        if full_data is None: return
        self.data = full_data

        # extract values to be graphed
        total_steps_taken = full_data.shape[1]
        total_time = self.dt * total_steps_taken
        times = np.linspace(0, total_time, num=total_steps_taken)
        position, velocity, acceleration, left_voltage, right_voltage, _ = full_data

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
