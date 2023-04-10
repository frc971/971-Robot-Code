import sys

import numpy as np

from y2023.control_loops.python.graph_tools import *


def ThetaSegment(name, start, end):
    control = np.array([(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0])
    return ThetaSplineSegment(
        name=name,
        start=start,
        control1=control,
        control2=control,
        end=end,
    )


named_segments = []
points = {}

points['Neutral'] = np.array((np.pi, 0.0, 0.0))

points['GroundPickupBackConeUp'] = to_theta_with_circular_index_and_roll(
    -1.07774334, 0.39, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackConeUp",
        start=points['Neutral'],
        control1=np.array([3.170156, -0.561227]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackConeUp'],
        control_alpha_rolls=[(.95, np.pi / 2.0)],
    ))

points['GroundPickupBackConeDownBase'] = to_theta_with_circular_index_and_roll(
    -1.11487594, 0.24, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackConeDownBase",
        start=points['Neutral'],
        control1=np.array([3.170156, -0.561227]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackConeDownBase'],
        control_alpha_rolls=[(.95, np.pi / 2.0)],
    ))

points[
    'GroundPickupFrontConeDownBase'] = to_theta_with_circular_index_and_roll(
        0.30, 0.265, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupFrontConeDownBase",
        start=points['Neutral'],
        control1=np.array([3.495221564200401, 0.4737763579250964]),
        control2=np.array([4.110392601248856, 1.0424853539638115]),
        end=points['GroundPickupFrontConeDownBase'],
        control_alpha_rolls=[(.95, -np.pi / 2.0)],
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

points['ScoreBackLowConeDownTip'] = to_theta_with_circular_index_and_roll(
    -1.17422, 0.441203, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreBackLowConeDownTip",
        start=points['Neutral'],
        control1=np.array([3.0959727041167358, -0.48933188185224896]),
        control2=np.array([3.11854219540683, -1.0398000886366843]),
        end=points['ScoreBackLowConeDownTip'],
        control_alpha_rolls=[(0.20, 0.0), (.95, np.pi / 2.0)],
    ))

points['ScoreFrontLowConeDownTip'] = to_theta_with_circular_index_and_roll(
    0.327783, 0.430704, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontLowConeDownTip",
        start=points['Neutral'],
        control1=np.array([3.6217558044411176, 0.6335548380532725]),
        control2=np.array([4.2557660430407935, 1.0411926555706872]),
        end=points['ScoreFrontLowConeDownTip'],
        control_alpha_rolls=[(0.30, 0.0), (.95, np.pi / 2.0)],
    ))

points['ScoreBackMidConeDownTip'] = to_theta_with_circular_index_and_roll(
    -1.49, 0.818521, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreBackMidConeDownTip",
        start=points['Neutral'],
        control1=np.array([3.193704394908777, -0.46076706416611657]),
        control2=np.array([3.6421839688861786, -0.8129214904599373]),
        end=points['ScoreBackMidConeDownTip'],
        control_alpha_rolls=[(0.20, 0.0), (.95, -np.pi / 2.0)],
    ))

points[
    'ScoreBackMidConeDownTipPlaced'] = to_theta_with_circular_index_and_roll(
        -1.43, 0.65, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreBackMidConeDownTipPlaced",
        start=points['Neutral'],
        control1=np.array([3.193704394908777, -0.46076706416611657]),
        control2=np.array([3.6421839688861786, -0.8129214904599373]),
        end=points['ScoreBackMidConeDownTipPlaced'],
        control_alpha_rolls=[(0.20, 0.0), (.95, -np.pi / 2.0)],
    ))

named_segments.append(
    ThetaSegment(
        name="ScoreBackMidConeDownTipToScoreBackMidConeDownTipPlaced",
        start=points['ScoreBackMidConeDownTip'],
        end=points['ScoreBackMidConeDownTipPlaced'],
    ))

points['ScoreFrontMidConeDownTip'] = np.array(
    (6.37001629521978, 2.04450540030891, np.pi / 2.0))
#to_theta_with_circular_index_and_roll(
#0.708449, 0.869738, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontMidConeDownTip",
        start=points['Neutral'],
        control1=np.array([4.579377666056791, 0.3789471836198275]),
        control2=np.array([5.140992799899862, 1.5135884307866865]),
        end=points['ScoreFrontMidConeDownTip'],
        control_alpha_rolls=[(0.50, 0.0), (.95, np.pi / 2.0)],
    ))

points['ScoreFrontMidConeDownTipPlaced'] = np.array(
    (6.42001629521978, 2.30450540030891, np.pi / 2.0))

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontMidConeDownTipPlaced",
        start=points['Neutral'],
        control1=np.array([4.579377666056791, 0.3789471836198275]),
        control2=np.array([5.140992799899862, 1.5135884307866865]),
        end=points['ScoreFrontMidConeDownTipPlaced'],
        control_alpha_rolls=[(0.50, 0.0), (.95, np.pi / 2.0)],
    ))

named_segments.append(
    ThetaSegment(
        name="ScoreFrontMidConeDownTipToScoreFrontMidConeDownTipPlaced",
        start=points['ScoreFrontMidConeDownTip'],
        end=points['ScoreFrontMidConeDownTipPlaced'],
    ))

points['ScoreFrontHighConeDownTip'] = np.array(
    (7.07190783461154, 1.55094570328448, np.pi / 2.0))
#to_theta_with_circular_index_and_roll(
#0.708449, 0.869738, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontHighConeDownTip",
        start=points['Neutral'],
        control1=np.array([4.579377666056791, 0.3789471836198275]),
        control2=np.array([5.140992799899862, 1.5135884307866865]),
        end=points['ScoreFrontHighConeDownTip'],
        control_alpha_rolls=[(0.50, 0.0), (.95, np.pi / 2.0)],
    ))

points['ScoreFrontHighConeDownTipPlaced'] = np.array(
    (6.93190783461154, 1.80094570328448, np.pi / 2.0))

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontHighConeDownTipPlaced",
        start=points['Neutral'],
        control1=np.array([5.997741842590495, 1.8354263885166913]),
        control2=np.array([6.141018843972322, 1.0777341552037734]),
        end=points['ScoreFrontHighConeDownTipPlaced'],
        control_alpha_rolls=[(0.30, 0.0), (.95, np.pi / 2.0)],
    ))

named_segments.append(
    ThetaSegment(
        name="ScoreFrontHighConeDownTipToScoreFrontHighConeDownTipPlaced",
        start=points['ScoreFrontHighConeDownTip'],
        end=points['ScoreFrontHighConeDownTipPlaced'],
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
    -1.102, 0.28, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupBackCube",
        start=points['Neutral'],
        control1=np.array([3.153228, -0.497009]),
        control2=np.array([2.972776, -1.026820]),
        end=points['GroundPickupBackCube'],
        control_alpha_rolls=[(.9, -np.pi / 2.0)],
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
        control_alpha_rolls=[(.9, np.pi / 2.0)],
    ))

points['ScoreBackMidConeUp'] = to_theta_with_circular_index_and_roll(
    -1.45013, 1.00354, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToBackMidConeUpScore",
        start=points['Neutral'],
        control1=np.array([3.6130298244820453, -0.2781204657180023]),
        control2=np.array([3.804763224169111, -0.5179424890517237]),
        end=points['ScoreBackMidConeUp'],
        control_alpha_rolls=[(.95, np.pi / 2.0)],
    ))

points['ScoreBackLowConeUp'] = to_theta_with_circular_index_and_roll(
    -1.00472, 0.472615, np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToBackLowConeUpScore",
        start=points['Neutral'],
        control1=np.array([3.260123029490386, -0.5296803702636037]),
        control2=np.array([3.1249665389044283, -0.7810758529482493]),
        end=points['ScoreBackLowConeUp'],
        control_alpha_rolls=[(.95, np.pi / 2.0)],
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
    -1.37792406, 0.87332449, np.pi / 2.0, circular_index=1)

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
    -1.1200539, 1.325, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToHPPickupBackConeUp",
        start=points['Neutral'],
        control1=np.array([2.7014360412658567, -0.32490272351246796]),
        control2=np.array([0.8282769211604871, -1.8026615176254461]),
        end=points['HPPickupBackConeUp'],
        control_alpha_rolls=[(.9, np.pi / 2.0)],
        alpha_unitizer=np.matrix(
            f"{1.0 / 15.0} 0 0; 0 {1.0 / 15.0} 0; 0 0 {1.0 / 100.0}"),
    ))

points['HPPickupFrontConeUp'] = np.array(
    (5.16514378449353, 1.25, -np.pi / 2.0))
#        to_theta_with_circular_index_and_roll(
#    0.265749, 1.28332, -np.pi / 2.0, circular_index=1)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToHPPickupFrontConeUp",
        start=points['Neutral'],
        control1=np.array([3.7428100581067785, 0.3957215032198551]),
        control2=np.array([4.349242198354247, 0.8724403878333801]),
        end=points['HPPickupFrontConeUp'],
        control_alpha_rolls=[(.8, -np.pi / 2.0)],
        alpha_unitizer=np.matrix(
            f"{1.0 / 15.0} 0 0; 0 {1.0 / 15.0} 0; 0 0 {1.0 / 70.0}"),
    ))

points['ScoreFrontHighConeUp'] = to_theta_with_circular_index_and_roll(
    0.98810344, 1.31536719, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToFrontHighConeUpScore",
        start=points['Neutral'],
        control1=np.array([2.594244, 0.417442]),
        control2=np.array([1.51325, 0.679748]),
        end=points['ScoreFrontHighConeUp'],
        control_alpha_rolls=[(.95, -np.pi / 2.0)],
    ))

points['ScoreFrontMidConeUp'] = to_theta_with_circular_index_and_roll(
    0.64, 0.99, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToFrontMidConeUpScore",
        start=points['Neutral'],
        control1=np.array([3.0, 0.317442]),
        control2=np.array([2.9, 0.479748]),
        end=points['ScoreFrontMidConeUp'],
        control_alpha_rolls=[(.95, -np.pi / 2.0)],
    ))

points['Starting'] = np.array((np.pi, -0.15, 0.0))

points['ScoreFrontMidConeUpAuto'] = to_theta_with_circular_index_and_roll(
    0.58, 0.97, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="StartingToScoreFrontMidConeUpAuto",
        start=points['Starting'],
        control1=np.array([2.99620794024176, 0.23620211875551145]),
        control2=np.array([2.728197531599509, 0.5677148040671784]),
        end=points['ScoreFrontMidConeUpAuto'],
        control_alpha_rolls=[(.85, -np.pi / 2.0)],
        vmax=10.0,
        alpha_unitizer=np.matrix(
            f"{1.0 / 20.0} 0 0; 0 {1.0 / 25.0} 0; 0 0 {1.0 / 100.0}"),
    ))

named_segments.append(
    ThetaSplineSegment(
        name="ScoreFrontMidConeUpAutoToGroundPickupBackCube",
        start=points['ScoreFrontMidConeUpAuto'],
        control1=np.array([3.1869633311848187, 0.2812689595803919]),
        control2=np.array([2.906100237354555, -0.7760928122326023]),
        end=points['GroundPickupBackCube'],
        control_alpha_rolls=[(0.40, 0.0), (0.60, 0.0)],
    ))

points['ScoreFrontLowCube'] = to_theta_with_circular_index_and_roll(
    0.325603, 0.39, np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToScoreFrontLowCube",
        start=points['Neutral'],
        control1=np.array([3.338852196583635, 0.34968650009090885]),
        control2=np.array([4.28246270189025, 1.492916470137478]),
        end=points['ScoreFrontLowCube'],
        control_alpha_rolls=[(.9, np.pi / 2.0)],
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
        control_alpha_rolls=[(0.95, np.pi * 0.5)],
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
        control_alpha_rolls=[(0.95, np.pi * 0.5)],
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
    -1.102, 0.3712121, -np.pi / 2.0, circular_index=1)

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
        control_alpha_rolls=[(0.95, -np.pi * 0.5)],
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
        control_alpha_rolls=[(0.95, -np.pi * 0.5)],
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
    0.313099, 0.390, -np.pi / 2.0, circular_index=0)

named_segments.append(
    ThetaSplineSegment(
        name="NeutralToGroundPickupFrontConeUp",
        start=points['Neutral'],
        control1=np.array([3.153481004695907, 0.4827717171390571]),
        control2=np.array([4.107487625131798, 0.9935705415901082]),
        end=points['GroundPickupFrontConeUp'],
        control_alpha_rolls=[(.95, -np.pi / 2.0)],
    ))

points['ScoreFrontLowConeUp'] = to_theta_with_circular_index_and_roll(
    0.33, 0.42, -np.pi / 2.0, circular_index=0)

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

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupBackConeUpToGroundPickupBackConeDownBase",
        start=points["GroundPickupBackConeUp"],
        control1=np.array([2.9926247819374154, -1.4933529586884973]),
        control2=np.array([3.0109584409452284, -1.545647945665206]),
        end=points["GroundPickupBackConeDownBase"],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupBackCubeToGroundPickupBackConeUp",
        start=points["GroundPickupBackCube"],
        control1=np.array([3.0505709159476315, -1.2367645274118955]),
        control2=np.array([3.092073954938234, -1.2212460452853597]),
        end=points["GroundPickupBackConeUp"],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupBackCubeToGroundPickupBackConeDownBase",
        start=points["GroundPickupBackCube"],
        control1=np.array([3.1162986756987365, -1.277140627447758]),
        control2=np.array([3.040845013474888, -1.3101069219600996]),
        end=points["GroundPickupBackConeDownBase"],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupFrontConeUpToGroundPickupFrontConeDownBase",
        start=points["GroundPickupFrontConeUp"],
        control1=np.array([4.165611255924869, 1.776661759504955]),
        control2=np.array([4.158617821533932, 1.8192037605481532]),
        end=points["GroundPickupFrontConeDownBase"],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupFrontCubeToGroundPickupFrontConeUp",
        start=points["GroundPickupFrontCube"],
        control1=np.array([4.24343254606187, 1.5218945203478058]),
        control2=np.array([4.264613405238473, 1.4226736369442823]),
        end=points["GroundPickupFrontConeUp"],
        control_alpha_rolls=[],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupFrontCubeToGroundPickFrontCubeDownBase",
        start=points["GroundPickupFrontCube"],
        control1=np.array([4.291217306429481, 1.6211752281452299]),
        control2=np.array([4.329997918670176, 1.6541325057673302]),
        end=points["GroundPickupFrontConeDownBase"],
        control_alpha_rolls=[],
    ))

# Auto express spline...
named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupBackCubeToScoreFrontMidCube",
        start=points['ScoreFrontMidCube'],
        control1=np.array([3.2345111429709847, 0.45338639767112277]),
        control2=np.array([3.098240119468829, -0.46161157069783254]),
        end=points['GroundPickupBackCube'],
        control_alpha_rolls=[(0.40, 0.0), (0.60, 0.0)],
    ))

named_segments.append(
    ThetaSplineSegment(
        name="GroundPickupBackCubeToScoreFrontHighCube",
        start=points['ScoreFrontHighCube'],
        control1=np.array([2.7074513232200186, 0.20154350392334375]),
        control2=np.array([3.01714846217257, -0.6310437434614364]),
        end=points['GroundPickupBackCube'],
        control_alpha_rolls=[(0.40, 0.0), (0.60, 0.0)],
    ))

front_points = []
back_points = []
unnamed_segments = []
segments = named_segments + unnamed_segments

# This checks that all points are unique

seen_segments = []

for segment in segments:
    # check for equality of the start and end values

    if (segment.start.tolist(), segment.end.tolist()) in seen_segments:
        print("Repeated value")
        segment.Print(points)
        sys.exit(1)
    else:
        seen_segments.append((segment.start.tolist(), segment.end.tolist()))

seen_points = []

for point in points:
    if point in seen_points:
        print(f"Repeated value {point}")
        sys.exit(1)
    else:
        seen_points.append(point)
