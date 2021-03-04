from constants import *
import cairo
from color import Color, palette
from points import Points
from drawing_constants import *
from libspline import Spline, DistanceSpline, Trajectory

AXIS_MARGIN_SPACE = 40


class Graph():  # (TODO): Remove Computer Calculation
    def __init__(self, cr, mypoints):
        # Background Box
        set_color(cr, palette["WHITE"])
        cr.rectangle(-1.0 * SCREEN_SIZE, -0.5 * SCREEN_SIZE, SCREEN_SIZE,
                     SCREEN_SIZE * 0.6)
        cr.fill()

        cr.set_source_rgb(0, 0, 0)
        cr.rectangle(-1.0 * SCREEN_SIZE, -0.5 * SCREEN_SIZE, SCREEN_SIZE,
                     SCREEN_SIZE * 0.6)
        #Axis
        cr.move_to(-1.0 * SCREEN_SIZE + AXIS_MARGIN_SPACE,
                   -0.5 * SCREEN_SIZE + AXIS_MARGIN_SPACE)  # Y
        cr.line_to(-1.0 * SCREEN_SIZE + AXIS_MARGIN_SPACE,
                   0.1 * SCREEN_SIZE - 10)

        cr.move_to(-1.0 * SCREEN_SIZE + AXIS_MARGIN_SPACE,
                   -0.5 * SCREEN_SIZE + AXIS_MARGIN_SPACE)  # X
        cr.line_to(-10, -0.5 * SCREEN_SIZE + AXIS_MARGIN_SPACE)
        cr.stroke()

        skip = 2
        dT = 0.00505
        start = AXIS_MARGIN_SPACE - SCREEN_SIZE
        end = -2.0 * AXIS_MARGIN_SPACE
        height = 0.5 * (SCREEN_SIZE) - AXIS_MARGIN_SPACE
        zero = AXIS_MARGIN_SPACE - SCREEN_SIZE / 2.0
        legend_entries = {}
        if mypoints.getLibsplines():
            distanceSpline = DistanceSpline(mypoints.getLibsplines())
            traj = Trajectory(distanceSpline)
            mypoints.addConstraintsToTrajectory(traj)
            traj.Plan()
            XVA = traj.GetPlanXVA(dT)
            if XVA is not None:
                self.draw_x_axis(cr, start, height, zero, XVA, end)
                self.drawVelocity(cr, XVA, start, height, skip, zero, end,
                                  legend_entries)
                self.drawAcceleration(cr, XVA, start, height, skip, zero,
                                      AXIS_MARGIN_SPACE, end, legend_entries)
                self.drawVoltage(cr, XVA, start, height, skip, traj, zero, end,
                                 legend_entries)
                cr.set_source_rgb(0, 0, 0)
                cr.move_to(-1.0 * AXIS_MARGIN_SPACE, zero + height / 2.0)
                cr.line_to(AXIS_MARGIN_SPACE - SCREEN_SIZE,
                           zero + height / 2.0)
                self.drawLegend(cr, XVA, start, height, skip, zero, end,
                                legend_entries)
        cr.stroke()

    def connectLines(self, cr, points, color):
        for i in range(0, len(points) - 1):
            set_color(cr, color)
            cr.move_to(points[i][0], points[i][1])
            cr.line_to(points[i + 1][0], points[i + 1][1])
            cr.stroke()

    def draw_x_axis(self, cr, start, height, zero, xva, end):
        total_time = 0.00505 * len(xva[0])
        for k in np.linspace(0, 1, 11):
            self.tickMark(cr,
                          k * np.abs(start - end) + start, zero + height / 2.0,
                          10, palette["BLACK"])
            cr.move_to(k * np.abs(start - end) + start,
                       10 + zero + height / 2.0)
            txt_scale = SCREEN_SIZE / 1000.0
            display_text(cr, str(round(k * total_time, 3)), txt_scale,
                         txt_scale, 1.0 / txt_scale, 1.0 / txt_scale)
            cr.stroke()

    def tickMark(self, cr, x, y, height, COLOR):
        # X, Y is in the middle of the tick mark
        set_color(cr, COLOR)
        cr.move_to(x, y + (height / 2))
        cr.line_to(x, y - (height / 2))
        cr.stroke()

    def HtickMark(self, cr, x, y, width, COLOR):
        # X, Y is in the middle of the tick mark
        set_color(cr, COLOR)
        cr.move_to(x + (width / 2), y)
        cr.line_to(x - (width / 2), y)
        cr.stroke()

    def drawLegend(self, cr, xva, start, height, skip, zero, end,
                   legend_entries):
        step_size = (end - start) / len(legend_entries)
        margin_under_x_axis = height * 0.1
        for index, (name, color) in enumerate(legend_entries.items()):
            set_color(cr, color)
            cr.move_to(start + index * step_size, zero - margin_under_x_axis)
            txt_scale = SCREEN_SIZE / 900.0
            display_text(cr, name, txt_scale, txt_scale, 1.0 / txt_scale,
                         1.0 / txt_scale)

    def drawVelocity(self, cr, xva, start, height, skip, zero, end,
                     legend_entries):
        COLOR = palette["RED"]
        velocity = xva[1]
        n_timesteps = len(velocity)
        max_v = np.amax(velocity)
        spacing = np.abs(start - end) / float(n_timesteps)
        scaler = height / max_v
        cr.set_source_rgb(1, 0, 0)
        points = []
        for i in range(0, len(velocity)):
            if i % skip == 0:
                points.append([
                    start + (i * spacing),
                    zero + height / 2.0 + (velocity[i] * scaler / 2.0)
                ])
        self.connectLines(cr, points, COLOR)

        # draw axes marking
        for i in np.linspace(-1, 1, 11):
            self.HtickMark(cr, start, zero + i * height / 2.0 + height / 2.0,
                           10, palette["BLACK"])
            cr.set_source_rgb(1, 0, 0)
            cr.move_to(start + 5, zero + i * height / 2.0 + height / 2.0)
            txt_scale = SCREEN_SIZE / 1000.0
            display_text(cr, str(round(i * max_v, 2)), txt_scale, txt_scale,
                         1.0 / txt_scale, 1.0 / txt_scale)
            cr.stroke()

        # add entry to legend
        legend_entries["Velocity"] = COLOR

    def drawAcceleration(self, cr, xva, start, height, skip, zero, margin, end,
                         legend_entries):
        COLOR = palette["BLUE"]
        accel = xva[2]
        max_a = np.amax(accel)
        min_a = np.amin(accel)
        n_timesteps = len(accel)
        spacing = np.abs(start - end) / float(n_timesteps)
        scaler = height / (max_a - min_a)
        cr.set_source_rgb(1, 0, 0)
        points = []
        for i in range(0, len(accel)):
            if i % skip == 0:
                points.append([
                    start + (i * spacing), zero + ((accel[i] - min_a) * scaler)
                ])
        self.connectLines(cr, points, COLOR)

        # draw axes marking
        for i in np.linspace(0, 1, 11):
            self.HtickMark(cr, -1.5 * margin, zero + i * height, 10,
                           palette["BLACK"])
            cr.set_source_rgb(0, 0, 1)
            cr.move_to(-1.2 * margin, zero + i * height)
            txt_scale = SCREEN_SIZE / 1000.0
            display_text(cr, str(round(i * (max_a - min_a) + min_a,
                                       2)), txt_scale, txt_scale,
                         1.0 / txt_scale, 1.0 / txt_scale)
            cr.stroke()

        # draw legend
        legend_entries["Acceleration"] = COLOR

    def drawVoltage(self, cr, xva, start, height, skip, traj, zero, end,
                    legend_entries):
        COLOR_LEFT = palette["GREEN"]
        COLOR_RIGHT = palette["CYAN"]
        poses = xva[0]
        n_timesteps = len(poses)
        spacing = np.abs(start - end) / float(n_timesteps)
        points_left = []
        points_right = []
        for i in range(0, len(poses)):
            if i % skip == 0:
                # libspline says the order is left-right
                voltage = traj.Voltage(poses[i])
                points_left.append([
                    start + (i * spacing),
                    zero + height / 2 + height * (voltage[0] / 24.0)
                ])
                points_right.append([
                    start + (i * spacing),
                    zero + height / 2 + height * (voltage[1] / 24.0)
                ])
        self.connectLines(cr, points_left, COLOR_LEFT)
        self.connectLines(cr, points_right, COLOR_RIGHT)

        for i in np.linspace(-1, 1, 7):
            self.HtickMark(cr, -1.0 * SCREEN_SIZE,
                           zero + i * height / 2.0 + height / 2.0, 10,
                           palette["BLACK"])
            cr.set_source_rgb(0, 1, 1)
            cr.move_to(-1.0 * SCREEN_SIZE,
                       zero + i * height / 2.0 + height / 2.0)
            txt_scale = SCREEN_SIZE / 1000.0
            display_text(cr, str(round(i * 12.0, 2)), txt_scale, txt_scale,
                         1.0 / txt_scale, 1.0 / txt_scale)
            cr.stroke()

        legend_entries["Left Voltage"] = COLOR_LEFT
        legend_entries["Right Voltage"] = COLOR_RIGHT
