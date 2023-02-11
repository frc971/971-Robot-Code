import numpy

from graph_tools import *

neutral = to_theta_with_circular_index(-0.2, 0.33, circular_index=-1)
zero = to_theta_with_circular_index(0.0, 0.0, circular_index=-1)

neutral_to_cone_1 = to_theta_with_circular_index(0.0, 0.7, circular_index=-1)
neutral_to_cone_2 = to_theta_with_circular_index(0.2, 0.5, circular_index=-1)
cone_pos = to_theta_with_circular_index(1.0, 0.4, circular_index=-1)

neutral_to_cone_perch_pos_1 = to_theta_with_circular_index(0.4,
                                                           1.0,
                                                           circular_index=-1)
neutral_to_cone_perch_pos_2 = to_theta_with_circular_index(0.7,
                                                           1.5,
                                                           circular_index=-1)
cone_perch_pos = to_theta_with_circular_index(1.0, 2.0, circular_index=-1)

points = []
front_points = []
back_points = []
unnamed_segments = []
named_segments = []

segments = [
    ThetaSplineSegment(neutral, neutral_to_cone_1, neutral_to_cone_2, cone_pos,
                       "NeutralToCone"),
    ThetaSplineSegment(neutral, neutral_to_cone_perch_pos_1,
                       neutral_to_cone_perch_pos_2, cone_perch_pos,
                       "NeutralToPerchedCone"),
]
