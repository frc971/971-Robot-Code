import numpy as np

from y2023.control_loops.python.graph_tools import *

named_segments = []
points = {}

points['Neutral'] = np.array((np.pi, 0.0, 0.0))

points['GroundPickupBackConeUp'] = to_theta_with_circular_index_and_roll(
    -1.07774334, 0.36308701, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackConeUp",
        start=points['Neutral'],
        control1=np.array([3.170156, -0.561227]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackConeUp'],
        control_alpha_rolls=[(0.30, 0.0), (.95, np.pi / 2.0)],
    ))

points['GroundPickupBackConeDown'] = to_theta_with_circular_index_and_roll(
    -1.11487594, 0.23140145, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackConeDown",
        start=points['Neutral'],
        control1=np.array([3.170156, -0.561227]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackConeDown'],
        control_alpha_rolls=[(0.30, 0.0), (.95, np.pi / 2.0)],
    ))

points['GroundPickupBackCube'] = to_theta_with_circular_index_and_roll(
    -1.102, 0.25, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackCube",
        start=points['Neutral'],
        control1=np.array([3.153228, -0.497009]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackCube'],
        control_alpha_rolls=[(0.7, 0.0), (.9, -np.pi / 2.0)],
    ))

points['GroundPickupFrontCube'] = to_theta_with_circular_index_and_roll(
    0.325603, 0.255189, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupFrontCube",
        start=points['Neutral'],
        control1=np.array([3.338852196583635, 0.34968650009090885]),
        control2=np.array([4.28246270189025, 1.492916470137478]),
        end=points['GroundPickupFrontCube'],
        control_alpha_rolls=[(0.4, 0.0), (.9, np.pi / 2.0)],
    ))

points['ScoreBackMidConeUpPos'] = to_theta_with_circular_index_and_roll(
    -1.41871454, 1.07476162, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToBackMidConeUpScore",
        start=points['Neutral'],
        control1=np.array([0.994244, -1.417442]),
        control2=np.array([1.711325, -0.679748]),
        end=points['ScoreBackMidConeUpPos'],
        control_alpha_rolls=[(0.40, 0.0), (.95, np.pi / 2.0)],
    ))

points['ScoreBackMidConeDownPos'] = to_theta_with_circular_index_and_roll(
    -1.37792406, 0.81332449, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToMidConeDownScore",
        start=points['Neutral'],
        control1=np.array([3.394572, -0.239378]),
        control2=np.array([3.654854, -0.626835]),
        end=points['ScoreBackMidConeDownPos'],
        control_alpha_rolls=[(0.40, 0.0), (.95, np.pi / 2.0)],
    ))

points['HPPickupBackConeUp'] = to_theta_with_circular_index_and_roll(
    -1.1050539, 1.31390128, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToHPPickupBackConeUp",
        start=points['Neutral'],
        control1=np.array([2.0, -0.239378]),
        control2=np.array([1.6, -0.626835]),
        end=points['HPPickupBackConeUp'],
        control_alpha_rolls=[(0.7, 0.0), (.9, np.pi / 2.0)],
    ))

points['ScoreFrontHighConeUpPos'] = to_theta_with_circular_index_and_roll(
    0.98810344, 1.37536719, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToFrontHighConeUpScore",
        start=points['Neutral'],
        control1=np.array([2.594244, 0.417442]),
        control2=np.array([1.51325, 0.679748]),
        end=points['ScoreFrontHighConeUpPos'],
        control_alpha_rolls=[(0.40, 0.0), (.95, -np.pi / 2.0)],
    ))

points['ScoreFrontMidConeUpPos'] = to_theta_with_circular_index_and_roll(
    0.43740453, 1.06330555, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToFrontMidConeUpScore",
        start=points['Neutral'],
        control1=np.array([3.0, 0.317442]),
        control2=np.array([2.9, 0.479748]),
        end=points['ScoreFrontMidConeUpPos'],
        control_alpha_rolls=[(0.40, 0.0), (.95, -np.pi / 2.0)],
    ))

points['ScoreFrontLowCube'] = to_theta_with_circular_index_and_roll(
    0.325603, 0.30, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontLowCube",
        start=points['Neutral'],
        control1=np.array([3.338852196583635, 0.34968650009090885]),
        control2=np.array([4.28246270189025, 1.492916470137478]),
        end=points['ScoreFrontLowCube'],
        control_alpha_rolls=[(0.4, 0.0), (.9, np.pi / 2.0)],
    ))

points['ScoreBackLowCube'] = to_theta_with_circular_index_and_roll(
    -1.102, 0.30, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreLowBackCube",
        start=points['Neutral'],
        control1=np.array([3.153228, -0.497009]),
        control2=np.array([2.972776, -1.026820]),
        end=points['ScoreBackLowCube'],
        control_alpha_rolls=[(0.7, 0.0), (.9, -np.pi / 2.0)],
    ))

points['ScoreFrontMidCube'] = to_theta_with_circular_index_and_roll(
    0.517846, 0.87, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontMidCube",
        start=points["Neutral"],
        control1=np.array([3.1310824883477952, 0.23591705727105095]),
        control2=np.array([3.0320025094685965, 0.43674789928668933]),
        end=points["ScoreFrontMidCube"],
        control_alpha_rolls=[(0.4, np.pi * 0.0), (0.95, np.pi * 0.5)],
    ))

points['ScoreFrontHighCube'] = to_theta_with_circular_index_and_roll(
    0.901437, 1.16, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontHighCube",
        start=points["Neutral"],
        control1=np.array([2.537484161662287, 0.059700523547219]),
        control2=np.array([2.449391812539668, 0.4141564369176016]),
        end=points["ScoreFrontHighCube"],
        control_alpha_rolls=[(0.4, np.pi * 0.0), (0.95, np.pi * 0.5)],
    ))

points['ScoreBackMidCube'] = to_theta_with_circular_index_and_roll(
    -1.27896, 0.84, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreBackMidCube",
        start=points["Neutral"],
        control1=np.array([3.3485646154655404, -0.4369603013926491]),
        control2=np.array([3.2653593368256995, -0.789587049476034]),
        end=points["ScoreBackMidCube"],
        control_alpha_rolls=[(0.3, -np.pi * 0.0), (0.95, -np.pi * 0.5)],
    ))

# TODO(austin): This doesn't produce the next line...
#points['ScoreBackHighCube'] = to_theta_with_circular_index_and_roll(
#    -1.60932, 1.16839, np.pi / 2.0, circular_index=0)
points['ScoreBackHighCube'] = np.array(
    (4.77284735761704, -1.19952193130714, -np.pi / 2.0))

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreBackHighCube",
        start=points["Neutral"],
        control1=np.array([3.6804854484103684, -0.3494541095053125]),
        control2=np.array([3.9889380578509517, -0.6637934755748516]),
        end=points["ScoreBackHighCube"],
        control_alpha_rolls=[(0.3, -np.pi * 0.0), (0.95, -np.pi * 0.5)],
    ))

points['ConeDownPos'] = to_theta_with_circular_index_and_roll(0.7,
                                                              0.11,
                                                              np.pi / 2.0,
                                                              circular_index=0)
named_segments.append(
    ThetaSplineSegment(
        name="NeutralToConeDown",
        start=points['Neutral'],
        control1=np.array([2.396694, 0.508020]),
        control2=np.array([2.874513, 0.933160]),
        end=points['ConeDownPos'],
        control_alpha_rolls=[(0.30, 0.0), (.95, np.pi / 2.0)],
    ))

front_points = []
back_points = []
unnamed_segments = []
segments = named_segments + unnamed_segments
