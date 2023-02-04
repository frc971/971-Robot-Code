import numpy

from graph_tools import *

tall_box_x = 0.411
tall_box_y = 0.125

short_box_x = 0.431
short_box_y = 0.082

ready_above_box = to_theta_with_circular_index(tall_box_x,
                                               tall_box_y + 0.08,
                                               circular_index=-1)
tall_box_grab = to_theta_with_circular_index(tall_box_x,
                                             tall_box_y,
                                             circular_index=-1)
short_box_grab = to_theta_with_circular_index(short_box_x,
                                              short_box_y,
                                              circular_index=-1)

# TODO(austin): Drive the front/back off the same numbers a bit better.
front_high_box = to_theta_with_circular_index(0.378, 2.46, circular_index=-1)
front_middle3_box = to_theta_with_circular_index(0.700,
                                                 2.125,
                                                 circular_index=-1.000000)
front_middle2_box = to_theta_with_circular_index(0.700,
                                                 2.268,
                                                 circular_index=-1)
front_middle1_box = to_theta_with_circular_index(0.800,
                                                 1.915,
                                                 circular_index=-1)
front_low_box = to_theta_with_circular_index(0.87, 1.572, circular_index=-1)
back_high_box = to_theta_with_circular_index(-0.75, 2.48, circular_index=0)
back_middle2_box = to_theta_with_circular_index(-0.700, 2.27, circular_index=0)
back_middle1_box = to_theta_with_circular_index(-0.800, 1.93, circular_index=0)
back_low_box = to_theta_with_circular_index(-0.87, 1.64, circular_index=0)

back_extra_low_box = to_theta_with_circular_index(-0.87,
                                                  1.52,
                                                  circular_index=0)

front_switch = to_theta_with_circular_index(0.88, 0.967, circular_index=-1)
back_switch = to_theta_with_circular_index(-0.88, 0.967, circular_index=-2)

neutral = to_theta_with_circular_index(0.0, 0.33, circular_index=-1)

up = to_theta_with_circular_index(0.0, 2.547, circular_index=-1)

front_switch_auto = to_theta_with_circular_index(0.750,
                                                 2.20,
                                                 circular_index=-1.000000)

duck = numpy.array([numpy.pi / 2.0 - 0.92, numpy.pi / 2.0 - 4.26])

starting = numpy.array([numpy.pi / 2.0 - 0.593329, numpy.pi / 2.0 - 3.749631])
vertical_starting = numpy.array([numpy.pi / 2.0, -numpy.pi / 2.0])

self_hang = numpy.array([numpy.pi / 2.0 - 0.191611, numpy.pi / 2.0])
partner_hang = numpy.array([numpy.pi / 2.0 - (-0.30), numpy.pi / 2.0])

above_hang = numpy.array([numpy.pi / 2.0 - 0.14, numpy.pi / 2.0 - (-0.165)])
below_hang = numpy.array([numpy.pi / 2.0 - 0.39, numpy.pi / 2.0 - (-0.517)])

up_c1 = to_theta((0.63, 1.17), circular_index=-1)
up_c2 = to_theta((0.65, 1.62), circular_index=-1)

front_high_box_c1 = to_theta((0.63, 1.04), circular_index=-1)
front_high_box_c2 = to_theta((0.50, 1.60), circular_index=-1)

front_middle2_box_c1 = to_theta((0.41, 0.83), circular_index=-1)
front_middle2_box_c2 = to_theta((0.52, 1.30), circular_index=-1)

front_middle1_box_c1 = to_theta((0.34, 0.82), circular_index=-1)
front_middle1_box_c2 = to_theta((0.48, 1.15), circular_index=-1)

#c1: (1.421433, -1.070254)
#c2: (1.434384, -1.057803
ready_above_box_c1 = numpy.array([1.480802, -1.081218])
ready_above_box_c2 = numpy.array([1.391449, -1.060331])

front_switch_c1 = numpy.array([1.903841, -0.622351])
front_switch_c2 = numpy.array([1.903841, -0.622351])

sparse_front_points = [
    (front_high_box, "FrontHighBox"),
    (front_middle2_box, "FrontMiddle2Box"),
    (front_middle3_box, "FrontMiddle3Box"),
    (front_middle1_box, "FrontMiddle1Box"),
    (front_low_box, "FrontLowBox"),
    (front_switch, "FrontSwitch"),
]  # yapf: disable

sparse_back_points = [
    (back_high_box, "BackHighBox"),
    (back_middle2_box, "BackMiddle2Box"),
    (back_middle1_box, "BackMiddle1Box"),
    (back_low_box, "BackLowBox"),
    (back_extra_low_box, "BackExtraLowBox"),
]  # yapf: disable

front_points, front_paths = expand_points(sparse_front_points, 0.06)
back_points, back_paths = expand_points(sparse_back_points, 0.06)

points = [(ready_above_box, "ReadyAboveBox"),
          (tall_box_grab, "TallBoxGrab"),
          (short_box_grab, "ShortBoxGrab"),
          (back_switch, "BackSwitch"),
          (neutral, "Neutral"),
          (up, "Up"),
          (above_hang, "AboveHang"),
          (below_hang, "BelowHang"),
          (self_hang, "SelfHang"),
          (partner_hang, "PartnerHang"),
          (front_switch_auto, "FrontSwitchAuto"),
          (starting, "Starting"),
          (duck, "Duck"),
          (vertical_starting, "VerticalStarting"),
] + front_points + back_points  # yapf: disable

duck_c1 = numpy.array([1.337111, -1.721008])
duck_c2 = numpy.array([1.283701, -1.795519])

ready_to_up_c1 = numpy.array([1.792962, 0.198329])
ready_to_up_c2 = numpy.array([1.792962, 0.198329])

front_switch_auto_c1 = numpy.array([1.792857, -0.372768])
front_switch_auto_c2 = numpy.array([1.861885, -0.273664])

# We need to define critical points so we can create paths connecting them.
# TODO(austin): Attach velocities to the slow ones.
ready_to_back_low_c1 = numpy.array([2.524325, 0.046417])

neutral_to_back_low_c1 = numpy.array([2.381942, -0.070220])

tall_to_back_low_c1 = numpy.array([2.603918, 0.088298])
tall_to_back_low_c2 = numpy.array([1.605624, 1.003434])

tall_to_back_high_c2 = numpy.array([1.508610, 0.946147])

# If true, only plot the first named segment
isolate = False

long_alpha_unitizer = numpy.matrix([[1.0 / 17.0, 0.0], [0.0, 1.0 / 17.0]])

neutral_to_back_c1 = numpy.array([0.702527, -2.618276])
neutral_to_back_c2 = numpy.array([0.526914, -3.109691])

named_segments = [
    ThetaSplineSegment(neutral, neutral_to_back_c1, neutral_to_back_c2,
                       back_switch, "BackSwitch"),
    ThetaSplineSegment(neutral,
                       neutral_to_back_low_c1,
                       tall_to_back_high_c2,
                       back_high_box,
                       "NeutralBoxToHigh",
                       alpha_unitizer=long_alpha_unitizer),
    ThetaSplineSegment(neutral, neutral_to_back_low_c1, tall_to_back_high_c2,
                       back_middle2_box, "NeutralBoxToMiddle2",
                       long_alpha_unitizer),
    ThetaSplineSegment(neutral, neutral_to_back_low_c1, tall_to_back_low_c2,
                       back_middle1_box, "NeutralBoxToMiddle1",
                       long_alpha_unitizer),
    ThetaSplineSegment(neutral, neutral_to_back_low_c1, tall_to_back_low_c2,
                       back_low_box, "NeutralBoxToLow", long_alpha_unitizer),
    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1,
                       tall_to_back_high_c2, back_high_box, "ReadyBoxToHigh",
                       long_alpha_unitizer),
    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1,
                       tall_to_back_high_c2, back_middle2_box,
                       "ReadyBoxToMiddle2", long_alpha_unitizer),
    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1,
                       tall_to_back_low_c2, back_middle1_box,
                       "ReadyBoxToMiddle1", long_alpha_unitizer),
    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1,
                       tall_to_back_low_c2, back_low_box, "ReadyBoxToLow",
                       long_alpha_unitizer),
    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1,
                       tall_to_back_high_c2, back_high_box, "ShortBoxToHigh",
                       long_alpha_unitizer),
    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1,
                       tall_to_back_high_c2, back_middle2_box,
                       "ShortBoxToMiddle2", long_alpha_unitizer),
    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1,
                       tall_to_back_low_c2, back_middle1_box,
                       "ShortBoxToMiddle1", long_alpha_unitizer),
    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1,
                       tall_to_back_low_c2, back_low_box, "ShortBoxToLow",
                       long_alpha_unitizer),
    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1,
                       tall_to_back_high_c2, back_high_box, "TallBoxToHigh",
                       long_alpha_unitizer),
    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1,
                       tall_to_back_high_c2, back_middle2_box,
                       "TallBoxToMiddle2", long_alpha_unitizer),
    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1, tall_to_back_low_c2,
                       back_middle1_box, "TallBoxToMiddle1",
                       long_alpha_unitizer),
    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1, tall_to_back_low_c2,
                       back_low_box, "TallBoxToLow", long_alpha_unitizer),
    SplineSegment(neutral, ready_above_box_c1, ready_above_box_c2,
                  ready_above_box, "ReadyToNeutral"),
    XYSegment(ready_above_box, tall_box_grab, "ReadyToTallBox", vmax=6.0),
    XYSegment(ready_above_box, short_box_grab, "ReadyToShortBox", vmax=6.0),
    XYSegment(tall_box_grab, short_box_grab, "TallToShortBox", vmax=6.0),
    SplineSegment(neutral, ready_above_box_c1, ready_above_box_c2,
                  tall_box_grab, "TallToNeutral"),
    SplineSegment(neutral, ready_above_box_c1, ready_above_box_c2,
                  short_box_grab, "ShortToNeutral"),
    SplineSegment(neutral, up_c1, up_c2, up, "NeutralToUp"),
    SplineSegment(neutral, front_high_box_c1, front_high_box_c2,
                  front_high_box, "NeutralToFrontHigh"),
    SplineSegment(neutral, front_middle2_box_c1, front_middle2_box_c2,
                  front_middle2_box, "NeutralToFrontMiddle2"),
    SplineSegment(neutral, front_middle1_box_c1, front_middle1_box_c2,
                  front_middle1_box, "NeutralToFrontMiddle1"),
]

unnamed_segments = [
    SplineSegment(neutral, front_switch_auto_c1, front_switch_auto_c2,
                  front_switch_auto),
    SplineSegment(tall_box_grab, ready_to_up_c1, ready_to_up_c2, up),
    SplineSegment(short_box_grab, ready_to_up_c1, ready_to_up_c2, up),
    SplineSegment(ready_above_box, ready_to_up_c1, ready_to_up_c2, up),
    ThetaSplineSegment(duck, duck_c1, duck_c2, neutral),
    SplineSegment(neutral, front_switch_c1, front_switch_c2, front_switch),
    XYSegment(ready_above_box, front_low_box),
    XYSegment(ready_above_box, front_switch),
    XYSegment(ready_above_box, front_middle1_box),
    XYSegment(ready_above_box, front_middle2_box),
    XYSegment(ready_above_box, front_middle3_box),
    SplineSegment(ready_above_box, ready_to_up_c1, ready_to_up_c2,
                  front_high_box),
    AngleSegment(starting, vertical_starting),
    AngleSegment(vertical_starting, neutral),
    XYSegment(neutral, front_low_box),
    XYSegment(up, front_high_box),
    XYSegment(up, front_middle2_box),
    XYSegment(front_middle3_box, up),
    XYSegment(front_middle3_box, front_high_box),
    XYSegment(front_middle3_box, front_middle2_box),
    XYSegment(front_middle3_box, front_middle1_box),
    XYSegment(up, front_middle1_box),
    XYSegment(up, front_low_box),
    XYSegment(front_high_box, front_middle2_box),
    XYSegment(front_high_box, front_middle1_box),
    XYSegment(front_high_box, front_low_box),
    XYSegment(front_middle2_box, front_middle1_box),
    XYSegment(front_middle2_box, front_low_box),
    XYSegment(front_middle1_box, front_low_box),
    XYSegment(front_switch, front_low_box),
    XYSegment(front_switch, up),
    XYSegment(front_switch, front_high_box),
    AngleSegment(up, back_high_box),
    AngleSegment(up, back_middle2_box),
    AngleSegment(up, back_middle1_box),
    AngleSegment(up, back_low_box),
    XYSegment(back_high_box, back_middle2_box),
    XYSegment(back_high_box, back_middle1_box),
    XYSegment(back_high_box, back_low_box),
    XYSegment(back_middle2_box, back_middle1_box),
    XYSegment(back_middle2_box, back_low_box),
    XYSegment(back_middle1_box, back_low_box),
    AngleSegment(up, above_hang),
    AngleSegment(above_hang, below_hang),
    AngleSegment(up, below_hang),
    AngleSegment(up, self_hang),
    AngleSegment(up, partner_hang),
] + front_paths + back_paths

segments = []
if isolate:
    segments += named_segments[:isolate]
else:
    segments += named_segments + unnamed_segments