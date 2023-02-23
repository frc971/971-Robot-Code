import numpy as np

from y2023.control_loops.python.graph_tools import *

neutral = to_theta_with_circular_index_and_roll(joint_center[0],
                                                joint_center[1] + l2 - l1,
                                                0.0,
                                                circular_index=1)
neutral_to_pickup_1 = np.array([2.396694, 0.508020])
neutral_to_pickup_2 = np.array([2.874513, 0.933160])
pickup_pos = to_theta_with_circular_index_and_roll(0.6,
                                                   0.4,
                                                   np.pi / 2.0,
                                                   circular_index=0)

neutral_to_pickup_control_alpha_rolls = [(0.33, 0.0), (.95, np.pi / 2.0)]

neutral_to_score_1 = np.array([0.994244, -1.417442])
neutral_to_score_2 = np.array([1.711325, -0.679748])

score_pos = to_theta_with_circular_index_and_roll(-1.0,
                                                  1.2,
                                                  np.pi / 2.0,
                                                  circular_index=0)
neutral_to_score_control_alpha_rolls = [(0.33, 0.0), (.95, np.pi / 2.0)]

# TODO(Max): Add real paths for arm.
points = [(neutral, "NeutralPos"), (pickup_pos, "PickupPos"),
          (score_pos, "ScorePos")]
front_points = []
back_points = []
unnamed_segments = []
named_segments = [
    ThetaSplineSegment("NeutralToPickup", neutral, neutral_to_pickup_1,
                       neutral_to_pickup_2, pickup_pos,
                       neutral_to_pickup_control_alpha_rolls),
    ThetaSplineSegment("NeutralToScore", neutral, neutral_to_score_1,
                       neutral_to_score_2, score_pos,
                       neutral_to_score_control_alpha_rolls),
]

segments = named_segments + unnamed_segments
