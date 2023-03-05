import numpy as np

from y2023.control_loops.python.graph_tools import *

named_segments = []
points = {}

points['Neutral'] = np.array((np.pi, 0.0, 0.0))

points['GroundPickupBackConeUp'] = to_theta_with_circular_index_and_roll(
    -1.07774334, 0.40, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackConeUp",
        start=points['Neutral'],
        control1=np.array([3.170156, -0.561227]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackConeUp'],
        control_alpha_rolls=[(0.30, 0.0), (.95, np.pi / 2.0)],
    ))

points['GroundPickupBackConeDownBase'] = to_theta_with_circular_index_and_roll(
    -1.11487594, 0.25, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackConeDownBase",
        start=points['Neutral'],
        control1=np.array([3.170156, -0.561227]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackConeDownBase'],
        control_alpha_rolls=[(0.30, 0.0), (.95, np.pi / 2.0)],
    ))

points[
    'GroundPickupFrontConeDownBase'] = to_theta_with_circular_index_and_roll(
        0.263207, 0.24, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupFrontConeDownBase",
        start=points['Neutral'],
        control1=np.array([3.495221564200401, 0.4737763579250964]),
        control2=np.array([4.110392601248856, 1.0424853539638115]),
        end=points['GroundPickupFrontConeDownBase'],
        control_alpha_rolls=[(0.30, 0.0), (.95, -np.pi / 2.0)],
    ))

points['ScoreFrontLowConeDownBase'] = to_theta_with_circular_index_and_roll(
    0.328533, 0.40, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontLowConeDownBase",
        start=points['Neutral'],
        control1=np.array([3.153481004695907, 0.4827717171390571]),
        control2=np.array([4.107487625131798, 0.9935705415901082]),
        end=points['ScoreFrontLowConeDownBase'],
        control_alpha_rolls=[(0.30, 0.0), (.95, -np.pi / 2.0)],
    ))

points['ScoreFrontMidConeDownBase'] = to_theta_with_circular_index_and_roll(
    0.697179, 0.88, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontMidConeDownBase",
        start=points['Neutral'],
        control1=np.array([3.2296966803523395, 0.4274365560093907]),
        control2=np.array([3.111677631381042, 0.6783534686461494]),
        end=points['ScoreFrontMidConeDownBase'],
        control_alpha_rolls=[(0.30, 0.0), (.95, -np.pi / 2.0)],
    ))

points['ScoreFrontHighConeDownBase'] = to_theta_with_circular_index_and_roll(
    1.04686, 1.13243, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontHighConeDownBase",
        start=points['Neutral'],
        control1=np.array([2.7653359284612185, 0.3091554519868296]),
        control2=np.array([2.6035409027556344, 0.5009078441624968]),
        end=points['ScoreFrontHighConeDownBase'],
        control_alpha_rolls=[(0.30, 0.0), (.95, -np.pi / 2.0)],
    ))

points['GroundPickupBackCube'] = to_theta_with_circular_index_and_roll(
    -1.102, 0.30, -np.pi / 2.0, circular_index=1)

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

points['ScoreBackMidConeUp'] = to_theta_with_circular_index_and_roll(
    -1.33013, 1.08354, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToBackMidConeUpScore",
        start=points['Neutral'],
        control1=np.array([3.6130298244820453, -0.2781204657180023]),
        control2=np.array([3.804763224169111, -0.5179424890517237]),
        end=points['ScoreBackMidConeUp'],
        control_alpha_rolls=[(0.40, 0.0), (.95, np.pi / 2.0)],
    ))

points['ScoreBackLowConeUp'] = to_theta_with_circular_index_and_roll(
    -1.00472, 0.672615, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToBackLowConeUpScore",
        start=points['Neutral'],
        control1=np.array([3.260123029490386, -0.5296803702636037]),
        control2=np.array([3.1249665389044283, -0.7810758529482493]),
        end=points['ScoreBackLowConeUp'],
        control_alpha_rolls=[(0.40, 0.0), (.95, np.pi / 2.0)],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupBackConeUpToBackLowConeUpScore",
        start=points['GroundPickupBackConeUp'],
        control1=np.array([2.943017165830755, -1.3740647485244808]),
        control2=np.array([2.941104610508278, -1.2434759967435083]),
        end=points['ScoreBackLowConeUp'],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="ScoreBackLowConeUpToScoreBackMidConeUp",
        start=points['ScoreBackLowConeUp'],
        control1=np.array([3.2930271753937728, -0.9256552441650734]),
        control2=np.array([3.6425461598470568, -0.8085366888146934]),
        end=points['ScoreBackMidConeUp'],
        control_alpha_rolls=[],
    ))

points['ScoreBackMidConeDownBase'] = to_theta_with_circular_index_and_roll(
    -1.37792406, 0.81332449, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToMidConeDownBaseScore",
        start=points['Neutral'],
        control1=np.array([3.394572, -0.239378]),
        control2=np.array([3.654854, -0.626835]),
        end=points['ScoreBackMidConeDownBase'],
        control_alpha_rolls=[(0.40, 0.0), (.95, np.pi / 2.0)],
    ))

points['ScoreBackLowConeDownBase'] = to_theta_with_circular_index_and_roll(
    -1.06372, 0.442764, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToLowConeDownBaseScore",
        start=points['Neutral'],
        control1=np.array([2.8613439132427896, -0.5868069120126034]),
        control2=np.array([2.9041434685529923, -1.240030040719494]),
        end=points['ScoreBackLowConeDownBase'],
        control_alpha_rolls=[(0.40, 0.0), (.95, np.pi / 2.0)],
    ))

points['HPPickupBackConeUp'] = to_theta_with_circular_index_and_roll(
    -1.1050539, 1.34, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToHPPickupBackConeUp",
        start=points['Neutral'],
        control1=np.array([2.0, -0.239378]),
        control2=np.array([1.6, -0.626835]),
        end=points['HPPickupBackConeUp'],
        control_alpha_rolls=[(0.7, 0.0), (.9, np.pi / 2.0)],
    ))

points['HPPickupFrontConeUp'] = np.array(
    (5.16514378449353, 1.26, -np.pi / 2.0))
#        to_theta_with_circular_index_and_roll(
#    0.265749, 1.28332, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToHPPickupFrontConeUp",
        start=points['Neutral'],
        control1=np.array([4.068204933788692, -0.05440997896697275]),
        control2=np.array([4.861911360838861, -0.03790410600482508]),
        end=points['HPPickupFrontConeUp'],
        control_alpha_rolls=[(0.7, 0.0), (.9, -np.pi / 2.0)],
    ))

points['ScoreFrontHighConeUp'] = to_theta_with_circular_index_and_roll(
    0.98810344, 1.37536719, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToFrontHighConeUpScore",
        start=points['Neutral'],
        control1=np.array([2.594244, 0.417442]),
        control2=np.array([1.51325, 0.679748]),
        end=points['ScoreFrontHighConeUp'],
        control_alpha_rolls=[(0.40, 0.0), (.95, -np.pi / 2.0)],
    ))

points['ScoreFrontMidConeUp'] = to_theta_with_circular_index_and_roll(
    0.43740453, 1.06330555, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToFrontMidConeUpScore",
        start=points['Neutral'],
        control1=np.array([3.0, 0.317442]),
        control2=np.array([2.9, 0.479748]),
        end=points['ScoreFrontMidConeUp'],
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

named_segments.append(
    ThetaSplineSegment(
        name="ScoreFrontLowCubeToScoreFrontMidCube",
        start=points["ScoreFrontLowCube"],
        control1=np.array([3.8237323383577078, 1.2979562720646056]),
        control2=np.array([3.63484177908944, 1.008850428344438]),
        end=points["ScoreFrontMidCube"],
        control_alpha_rolls=[],
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

named_segments.append(
    ThetaSplineSegment(
        name="ScoreFrontMidCubeToScoreFrontHighCube",
        start=points["ScoreFrontMidCube"],
        control1=np.array([2.9229652375897004, 0.7771801809056819]),
        control2=np.array([2.634276444896239, 0.5696525540129302]),
        end=points["ScoreFrontHighCube"],
        control_alpha_rolls=[],
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

named_segments.append(
    ThetaSplineSegment(
        name="ScoreBackLowCubeToScoreBackMidCube",
        start=points["ScoreBackLowCube"],
        control1=np.array([3.1075630474968694, -1.1675095818664531]),
        control2=np.array([3.3377520447373232, -1.1054408842366303]),
        end=points["ScoreBackMidCube"],
        control_alpha_rolls=[],
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

named_segments.append(
    ThetaSplineSegment(
        name="ScoreBackMidCubeToScoreBackHighCube",
        start=points["ScoreBackMidCube"],
        control1=np.array([4.03651864313893, -0.919229198708873]),
        control2=np.array([4.377346803653962, -1.0167608157302999]),
        end=points["ScoreBackHighCube"],
        control_alpha_rolls=[],
    ))

points['GroundPickupFrontConeUp'] = to_theta_with_circular_index_and_roll(
    0.313099, 0.380, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupFrontConeUp",
        start=points['Neutral'],
        control1=np.array([3.153481004695907, 0.4827717171390571]),
        control2=np.array([4.107487625131798, 0.9935705415901082]),
        end=points['GroundPickupFrontConeUp'],
        control_alpha_rolls=[(0.30, 0.0), (.95, -np.pi / 2.0)],
    ))

points['ScoreFrontLowConeUp'] = to_theta_with_circular_index_and_roll(
    0.349687, 0.468804, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontLowConeUp",
        start=points['Neutral'],
        control1=np.array([3.153481004695907, 0.4827717171390571]),
        control2=np.array([4.107487625131798, 0.9935705415901082]),
        end=points['ScoreFrontLowConeUp'],
        control_alpha_rolls=[(0.30, 0.0), (.95, -np.pi / 2.0)],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupFrontConeUpToScoreFrontLowConeUp",
        start=points['GroundPickupFrontConeUp'],
        control1=np.array([4.14454438793702, 1.680256664914554]),
        control2=np.array([4.159014136030164, 1.6617266432775355]),
        end=points['ScoreFrontLowConeUp'],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="ScoreFrontLowConeUpToScoreFrontMidConeUp",
        start=points['ScoreFrontLowConeUp'],
        control1=np.array([4.144103145250675, 1.3519566301042056]),
        control2=np.array([3.5357641970552223, 0.8105698293886593]),
        end=points['ScoreFrontMidConeUp'],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="ScoreFrontMidConeUpToScoreFrontHighConeUp",
        start=points['ScoreFrontMidConeUp'],
        control1=np.array([2.417981958011055, 0.48234108399079134]),
        control2=np.array([2.1651435746478045, 0.4937628492739232]),
        end=points['ScoreFrontHighConeUp'],
        control_alpha_rolls=[],
    ))

front_points = []
back_points = []
unnamed_segments = []
segments = named_segments + unnamed_segments
