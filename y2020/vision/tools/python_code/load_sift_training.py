#!/usr/bin/python3

import cv2
import glog
import numpy as np
import sys
import flatbuffers

import frc971.vision.sift.CameraCalibration as CameraCalibration
import frc971.vision.sift.Feature as Feature
import frc971.vision.sift.KeypointFieldLocation as KeypointFieldLocation
import frc971.vision.sift.TrainingData as TrainingData
import frc971.vision.sift.TrainingImage as TrainingImage
import frc971.vision.sift.TransformationMatrix as TransformationMatrix

import gflags
import glog

FLAGS = gflags.FLAGS


# Takes a 3x3 rotation matrix and 3x1 translation vector, and outputs 12
# element list, suitable for outputing to flatbuffer
def rot_and_trans_to_list(R, T):
    output_list = []
    for row in range(3):
        for col in range(3):
            output_list.append(R[row][col])
        output_list.append(T[row])

    output_list = output_list + [0., 0., 0., 1.]
    return output_list


def list_to_transformation_matrix(values, fbb):
    """Puts a list of values into an FBB TransformationMatrix."""

    TransformationMatrix.TransformationMatrixStartDataVector(fbb, len(values))
    for n in reversed(values):
        fbb.PrependFloat32(n)
    list_offset = fbb.EndVector()

    TransformationMatrix.TransformationMatrixStart(fbb)
    TransformationMatrix.TransformationMatrixAddData(fbb, list_offset)
    return TransformationMatrix.TransformationMatrixEnd(fbb)


def main():

    target_data_list = None
    camera_calib_list = None

    output_path = sys.argv[1]

    if len(sys.argv) > 2:
        if sys.argv[2] == "test":
            glog.info("Loading test data")
            import camera_definition_test
            import target_definition_test
            camera_calib_list = camera_definition_test.camera_list
            target_data_list = target_definition_test.compute_target_definition(
                camera_calib_list[0])
        else:
            glog.error("Unhandled arguments: '%s'" % sys.argv[2])
            quit()
    else:
        glog.debug("Loading target configuration data")
        import camera_definition
        import target_definition
        camera_calib_list = camera_definition.load_camera_definitions()
        target_data_list = target_definition.compute_target_definition(
            camera_definition.load_pi1_camera_params(camera_calib_list))

    glog.info("Writing file to %s", output_path)

    fbb = flatbuffers.Builder(0)

    images_vector = []

    # Iterate overall the training targets
    for target_data in target_data_list:

        features_vector = []
        # Iterate over all the keypoints
        for keypoint, keypoint_3d, descriptor in zip(
                target_data.keypoint_list, target_data.keypoint_list_3d,
                target_data.descriptor_list):

            # Build the Descriptor vector
            Feature.FeatureStartDescriptorVector(fbb, len(descriptor))
            for n in reversed(descriptor):
                assert n == round(n)
                fbb.PrependUint8(int(round(n)))
            descriptor_vector = fbb.EndVector()

            # Add all the components to the each Feature
            Feature.FeatureStart(fbb)
            Feature.FeatureAddDescriptor(fbb, descriptor_vector)
            Feature.FeatureAddX(fbb, keypoint.pt[0])
            Feature.FeatureAddY(fbb, keypoint.pt[1])
            Feature.FeatureAddSize(fbb, keypoint.size)
            Feature.FeatureAddAngle(fbb, keypoint.angle)
            Feature.FeatureAddResponse(fbb, keypoint.response)
            Feature.FeatureAddOctave(fbb, keypoint.octave)

            keypoint_3d_location = KeypointFieldLocation.CreateKeypointFieldLocation(
                fbb, keypoint_3d[0], keypoint_3d[1], keypoint_3d[2])

            Feature.FeatureAddFieldLocation(fbb, keypoint_3d_location)

            features_vector.append(Feature.FeatureEnd(fbb))

        # Create the field_to_target TransformationMatrix
        field_to_target_list = rot_and_trans_to_list(
            target_data.target_rotation, target_data.target_position)
        TransformationMatrix.TransformationMatrixStartDataVector(
            fbb, len(field_to_target_list))
        for n in reversed(field_to_target_list):
            fbb.PrependFloat32(n)
        field_to_target_offset = fbb.EndVector()

        TransformationMatrix.TransformationMatrixStart(fbb)
        TransformationMatrix.TransformationMatrixAddData(
            fbb, field_to_target_offset)
        transformation_mat_offset = TransformationMatrix.TransformationMatrixEnd(
            fbb)

        # Create the TrainingImage feature vector
        TrainingImage.TrainingImageStartFeaturesVector(fbb,
                                                       len(features_vector))
        for feature in reversed(features_vector):
            fbb.PrependUOffsetTRelative(feature)
        features_vector_offset = fbb.EndVector()

        # Add the TrainingImage data
        TrainingImage.TrainingImageStart(fbb)
        TrainingImage.TrainingImageAddFeatures(fbb, features_vector_offset)
        TrainingImage.TrainingImageAddFieldToTarget(fbb,
                                                    transformation_mat_offset)
        TrainingImage.TrainingImageAddTargetPointX(
            fbb, target_data.target_point_2d[0][0][0])
        TrainingImage.TrainingImageAddTargetPointY(
            fbb, target_data.target_point_2d[0][0][1])
        TrainingImage.TrainingImageAddTargetPointRadius(
            fbb, target_data.target_radius)
        images_vector.append(TrainingImage.TrainingImageEnd(fbb))

    # Create and add Training Data of all targets
    TrainingData.TrainingDataStartImagesVector(fbb, len(images_vector))
    for training_image in reversed(images_vector):
        fbb.PrependUOffsetTRelative(training_image)
    images_vector_table = fbb.EndVector()

    # Create camera calibration data
    camera_calibration_vector = []
    for camera_calib in camera_calib_list:
        fixed_extrinsics_list = rot_and_trans_to_list(
            camera_calib.camera_ext.R, camera_calib.camera_ext.T)
        fixed_extrinsics_vector = list_to_transformation_matrix(
            fixed_extrinsics_list, fbb)

        turret_extrinsics_vector = None
        if camera_calib.turret_ext is not None:
            turret_extrinsics_list = rot_and_trans_to_list(
                camera_calib.turret_ext.R, camera_calib.turret_ext.T)
            turret_extrinsics_vector = list_to_transformation_matrix(
                turret_extrinsics_list, fbb)

        # TODO: Need to add in distortion coefficients here
        # For now, just send camera paramter matrix (fx, fy, cx, cy)
        camera_int_list = camera_calib.camera_int.camera_matrix.ravel().tolist(
        )
        CameraCalibration.CameraCalibrationStartIntrinsicsVector(
            fbb, len(camera_int_list))
        for n in reversed(camera_int_list):
            fbb.PrependFloat32(n)
        intrinsics_vector = fbb.EndVector()

        dist_coeffs_list = camera_calib.camera_int.dist_coeffs.ravel().tolist()
        CameraCalibration.CameraCalibrationStartDistCoeffsVector(
            fbb, len(dist_coeffs_list))
        for n in reversed(dist_coeffs_list):
            fbb.PrependFloat32(n)
        dist_coeffs_vector = fbb.EndVector()

        node_name_offset = fbb.CreateString(camera_calib.node_name)
        CameraCalibration.CameraCalibrationStart(fbb)
        CameraCalibration.CameraCalibrationAddNodeName(fbb, node_name_offset)
        CameraCalibration.CameraCalibrationAddTeamNumber(
            fbb, camera_calib.team_number)
        CameraCalibration.CameraCalibrationAddIntrinsics(
            fbb, intrinsics_vector)
        CameraCalibration.CameraCalibrationAddDistCoeffs(
            fbb, dist_coeffs_vector)
        CameraCalibration.CameraCalibrationAddFixedExtrinsics(
            fbb, fixed_extrinsics_vector)
        if turret_extrinsics_vector is not None:
            CameraCalibration.CameraCalibrationAddTurretExtrinsics(
                fbb, turret_extrinsics_vector)
        camera_calibration_vector.append(
            CameraCalibration.CameraCalibrationEnd(fbb))

    TrainingData.TrainingDataStartCameraCalibrationsVector(
        fbb, len(camera_calibration_vector))
    for camera_calibration in reversed(camera_calibration_vector):
        fbb.PrependUOffsetTRelative(camera_calibration)
    camera_calibration_vector_table = fbb.EndVector()

    # Fill out TrainingData
    TrainingData.TrainingDataStart(fbb)
    TrainingData.TrainingDataAddImages(fbb, images_vector_table)
    TrainingData.TrainingDataAddCameraCalibrations(
        fbb, camera_calibration_vector_table)
    fbb.Finish(TrainingData.TrainingDataEnd(fbb))

    bfbs = fbb.Output()

    output_prefix = [
        b'#ifndef Y2020_VISION_TOOLS_PYTHON_CODE_TRAINING_DATA_H_',
        b'#define Y2020_VISION_TOOLS_PYTHON_CODE_TRAINING_DATA_H_',
        b'#include <stdint.h>',
        b'#include "absl/types/span.h"',
        b'namespace frc971 {',
        b'namespace vision {',
        b'inline absl::Span<const uint8_t> SiftTrainingData() {',
    ]
    output_suffix = [
        b'  return absl::Span<const uint8_t>(reinterpret_cast<const uint8_t *>(kData), sizeof(kData));',
        b'}',
        b'}  // namespace vision',
        b'}  // namespace frc971',
        b'#endif  // Y2020_VISION_TOOLS_PYTHON_CODE_TRAINING_DATA_H_',
    ]

    # Write out the header file
    with open(output_path, 'wb') as output:
        for line in output_prefix:
            output.write(line)
            output.write(b'\n')
        output.write(b'alignas(64) static constexpr char kData[] = "')
        for byte in fbb.Output():
            output.write(b'\\x' + (b'%x' % byte).zfill(2))
        output.write(b'";\n')
        for line in output_suffix:
            output.write(line)
            output.write(b'\n')


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    main()
