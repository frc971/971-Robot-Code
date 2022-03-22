#!/usr/bin/python3

import camera_definition
import cv2
import flatbuffers
import glog
import numpy as np
import sys

import frc971.vision.calibration.CalibrationData as CalibrationData
import frc971.vision.calibration.CameraCalibration as CameraCalibration
import frc971.vision.calibration.TransformationMatrix as TransformationMatrix

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
    camera_calib_list = camera_definition.load_camera_definitions()

    output_path = sys.argv[1]
    glog.debug("Writing file to %s", output_path)

    fbb = flatbuffers.Builder(0)

    images_vector = []

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

    CalibrationData.CalibrationDataStartCameraCalibrationsVector(
        fbb, len(camera_calibration_vector))
    for camera_calibration in reversed(camera_calibration_vector):
        fbb.PrependUOffsetTRelative(camera_calibration)
    camera_calibration_vector_table = fbb.EndVector()

    # Fill out TrainingData
    CalibrationData.CalibrationDataStart(fbb)
    CalibrationData.CalibrationDataAddCameraCalibrations(
        fbb, camera_calibration_vector_table)
    fbb.Finish(CalibrationData.CalibrationDataEnd(fbb))

    bfbs = fbb.Output()

    output_prefix = [
        b'#ifndef Y2022_VISION_CALIBRATION_DATA_H_',
        b'#define Y2022_VISION_CALIBRATION_DATA_H_',
        b'#include <stdint.h>',
        b'#include "absl/types/span.h"',
        b'namespace frc971 {',
        b'namespace vision {',
        b'inline absl::Span<const uint8_t> CalibrationData() {',
    ]
    output_suffix = [
        b'  return absl::Span<const uint8_t>(reinterpret_cast<const uint8_t *>(kData), sizeof(kData));',
        b'}',
        b'}  // namespace vision',
        b'}  // namespace frc971',
        b'#endif  // Y2022_VISION_CALIBRATION_DATA_H_',
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
