import numpy as np

from y2023.control_loops.python.graph_tools import *

neutral = to_theta_with_circular_index_and_roll(joint_center[0],
                                                joint_center[1] + l2 - l1,
                                                0.0,
                                                circular_index=1)

# NeutralToGroundPickupBackConeUp
neutral_to_cone_up_1 = np.array([3.170156, -0.561227])
neutral_to_cone_up_2 = np.array([2.972776, -1.026820])
ground_pickup_back_cone_up = to_theta_with_circular_index_and_roll(
    -0.913162844198605, 0.35, np.pi / 2.0, circular_index=1)

# NeutralToGroundPickupBackConeDown
neutral_to_ground_pickup_back_cone_down_1 = np.array([3.170156, -0.561227])
neutral_to_ground_pickup_back_cone_down_2 = np.array([2.972776, -1.026820])
ground_pickup_back_cone_down = to_theta_with_circular_index_and_roll(
    -0.95, 0.24, np.pi / 2.0, circular_index=1)

# NeutralToBackMidConeUpScore
neutral_to_score_back_mid_cone_up_1 = np.array([0.994244, -1.417442])
neutral_to_score_back_mid_cone_up_2 = np.array([1.711325, -0.679748])
score_back_mid_cone_up_pos = to_theta_with_circular_index_and_roll(
    -1.255555, 1.10, np.pi / 2.0, circular_index=0)

# NeutralToMidConeDownScore
neutral_to_score_mid_cone_down_1 = np.array([3.394572, -0.239378])
neutral_to_score_mid_cone_down_2 = np.array([3.654854, -0.626835])
score_mid_cone_down_pos = to_theta_with_circular_index_and_roll(
    -1.23, 0.74, np.pi / 2.0, circular_index=1)

# NeutralToMidConeDownScore
neutral_to_hp_pickup_back_cone_up_1 = np.array([2.0, -0.239378])
neutral_to_hp_pickup_back_cone_up_2 = np.array([1.6, -0.626835])
neutral_to_hp_pickup_back_cone_up_alpha_rolls = [
    (0.7, 0.0),
    (.9, np.pi / 2.0),
]
hp_pickup_back_cone_up = to_theta_with_circular_index_and_roll(
    -0.94, 1.31, np.pi / 2.0, circular_index=0)

# NeutralToFrontHighConeUpScore
neutral_to_score_front_high_cone_up_1 = np.array([2.594244, 0.417442])
neutral_to_score_front_high_cone_up_2 = np.array([1.51325, 0.679748])
score_front_high_cone_up_pos = to_theta_with_circular_index_and_roll(
    0.87, 1.26, -np.pi / 2.0, circular_index=0)

# NeutralToFrontMidConeUpScore
neutral_to_score_front_mid_cone_up_1 = np.array([3.0, 0.317442])
neutral_to_score_front_mid_cone_up_2 = np.array([2.9, 0.479748])
score_front_mid_cone_up_pos = to_theta_with_circular_index_and_roll(
    0.34, 0.93, -np.pi / 2.0, circular_index=0)

neutral_to_cone_down_1 = np.array([2.396694, 0.508020])
neutral_to_cone_down_2 = np.array([2.874513, 0.933160])
cone_down_pos = to_theta_with_circular_index_and_roll(0.7,
                                                      0.11,
                                                      np.pi / 2.0,
                                                      circular_index=0)

neutral_to_cube_1 = np.array([2.396694, 0.508020])
neutral_to_cube_2 = np.array([2.874513, 0.933160])

cube_pos = to_theta_with_circular_index_and_roll(0.7,
                                                 0.24,
                                                 np.pi / 2.0,
                                                 circular_index=0)

neutral_to_pickup_control_alpha_rolls = [
    (0.30, 0.0),
    (.95, np.pi / 2.0),
]

neutral_to_score_1 = np.array([0.994244, -1.417442])
neutral_to_score_2 = np.array([1.711325, -0.679748])

score_low_pos = to_theta_with_circular_index_and_roll(-(0.41 / 2 + 0.49),
                                                      0 + 0.05,
                                                      np.pi / 2.0,
                                                      circular_index=1)

neutral_to_score_low_2 = np.array([3.37926599, -0.73664663])

score_mid_cube_pos = to_theta_with_circular_index_and_roll(-(0.58 + 0.49),
                                                           0.6 + 0.05,
                                                           np.pi / 2.0,
                                                           circular_index=0)

score_high_cone_pos = to_theta_with_circular_index_and_roll(-1.01,
                                                            1.17 + 0.05,
                                                            np.pi / 2.0,
                                                            circular_index=0)

score_high_cube_pos = to_theta_with_circular_index_and_roll(-1.01,
                                                            0.90 + 0.05,
                                                            np.pi / 2.0,
                                                            circular_index=0)

neutral_to_back_score_control_alpha_rolls = [(0.40, 0.0), (.95, np.pi / 2.0)]
neutral_to_front_score_control_alpha_rolls = [(0.40, 0.0), (.95, -np.pi / 2.0)]

points = [(neutral, "NeutralPos"),
          (ground_pickup_back_cone_up, "GroundPickupBackConeUp"),
          (ground_pickup_back_cone_down, "GroundPickupBackConeDown"),
          (hp_pickup_back_cone_up, "HPPickupBackConeUp"),
          (cone_down_pos, "ConeDownPos"), (score_low_pos, "ScoreLowPos"),
          (score_back_mid_cone_up_pos, "ScoreBackMidConeUpPos"),
          (score_front_high_cone_up_pos, "ScoreFrontHighConeUpPos"),
          (score_front_mid_cone_up_pos, "ScoreFrontMidConeUpPos"),
          (score_mid_cone_down_pos, "ScoreBackMidConeDownPos"),
          (score_mid_cube_pos, "ScoreMidCubePos"),
          (score_high_cone_pos, "ScoreHighConePos"),
          (score_high_cube_pos, "ScoreHighCubePos"), (cube_pos, "CubePos")]
front_points = []
back_points = []
unnamed_segments = []
named_segments = [
    ThetaSplineSegment("NeutralToGroundPickupBackConeUp", neutral,
                       neutral_to_cone_up_1, neutral_to_cone_up_2,
                       ground_pickup_back_cone_up,
                       neutral_to_pickup_control_alpha_rolls),
    ThetaSplineSegment("NeutralToGroundPickupBackConeDown", neutral,
                       neutral_to_ground_pickup_back_cone_down_1,
                       neutral_to_ground_pickup_back_cone_down_2,
                       ground_pickup_back_cone_down,
                       neutral_to_pickup_control_alpha_rolls),
    ThetaSplineSegment("NeutralToHPPickupBackConeUp", neutral,
                       neutral_to_hp_pickup_back_cone_up_1,
                       neutral_to_hp_pickup_back_cone_up_2,
                       hp_pickup_back_cone_up,
                       neutral_to_hp_pickup_back_cone_up_alpha_rolls),
    ThetaSplineSegment("NeutralToFrontHighConeUpScore", neutral,
                       neutral_to_score_front_high_cone_up_1,
                       neutral_to_score_front_high_cone_up_2,
                       score_front_high_cone_up_pos,
                       neutral_to_front_score_control_alpha_rolls),
    ThetaSplineSegment("NeutralToFrontMidConeUpScore", neutral,
                       neutral_to_score_front_mid_cone_up_1,
                       neutral_to_score_front_mid_cone_up_2,
                       score_front_mid_cone_up_pos,
                       neutral_to_front_score_control_alpha_rolls),
    ThetaSplineSegment("NeutralToConeDown", neutral, neutral_to_cone_down_1,
                       neutral_to_cone_down_2, cone_down_pos,
                       neutral_to_pickup_control_alpha_rolls),
    ThetaSplineSegment("NeutralToCube", neutral, neutral_to_cube_1,
                       neutral_to_cube_2, cube_pos,
                       neutral_to_pickup_control_alpha_rolls),
    ThetaSplineSegment("NeutralToLowScore", neutral, neutral_to_score_1,
                       neutral_to_score_low_2, score_low_pos,
                       neutral_to_back_score_control_alpha_rolls),
    ThetaSplineSegment("NeutralToBackMidConeUpScore", neutral,
                       neutral_to_score_back_mid_cone_up_1,
                       neutral_to_score_back_mid_cone_up_2,
                       score_back_mid_cone_up_pos,
                       neutral_to_back_score_control_alpha_rolls),
    ThetaSplineSegment("NeutralToMidConeDownScore", neutral,
                       neutral_to_score_mid_cone_down_1,
                       neutral_to_score_mid_cone_down_2,
                       score_mid_cone_down_pos,
                       neutral_to_back_score_control_alpha_rolls),
    ThetaSplineSegment("NeutralToMidCubeScore", neutral, neutral_to_score_1,
                       neutral_to_score_2, score_mid_cube_pos,
                       neutral_to_back_score_control_alpha_rolls),
    ThetaSplineSegment("NeutralToHighConeScore", neutral, neutral_to_score_1,
                       neutral_to_score_2, score_high_cone_pos,
                       neutral_to_back_score_control_alpha_rolls),
    ThetaSplineSegment("NeutralToHighCubeScore", neutral, neutral_to_score_1,
                       neutral_to_score_2, score_high_cube_pos,
                       neutral_to_back_score_control_alpha_rolls),
]

segments = named_segments + unnamed_segments
