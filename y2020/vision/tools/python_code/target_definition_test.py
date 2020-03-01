import cv2
import numpy as np

import camera_definition_test
import define_training_data as dtd
import target_definition
import train_and_match as tam


def compute_target_definition():
    target_data_list = []
    target_data_test_1 = target_definition.TargetData(
        'test_images/train_power_port_red.png')

    target_data_test_1.extract_features()

    target_data_test_1.keypoint_list[0].pt = (0., 1.)

    kp_3d = []
    for i in range(len(target_data_test_1.keypoint_list)):
        kp_3d.append((i, i, i))

    target_data_test_1.keypoint_list_3d = np.asarray(
        np.float32(kp_3d)).reshape(-1, 1, 3)

    target_data_test_1.target_rotation = np.identity(3, np.double)
    target_data_test_1.target_position = np.array([0., 1., 2.])
    target_data_test_1.target_point_2d = np.array([10., 20.]).reshape(-1, 1, 2)
    target_data_test_1.target_radius = 32
    target_data_list.append(target_data_test_1)
    return target_data_list
