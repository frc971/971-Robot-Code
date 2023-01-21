import copy
import cv2
import flatbuffers
import glog
import json
import numpy as np
import os
import sys

import frc971.vision.calibration.CalibrationData as CalibrationData
import frc971.vision.calibration.CameraCalibration as CameraCalibration
import frc971.vision.calibration.TransformationMatrix as TransformationMatrix

import glog


def bazel_name_fix(filename, year):
    """Quick fix to naming games that happen with bazel"""
    ret_name = filename
    try:
        from bazel_tools.tools.python.runfiles import runfiles
        r = runfiles.Create()
        ret_name = r.Rlocation('org_frc971/y%s/vision/%s' % (year, filename))
    except:
        print("Failed bazel_name_fix")
        pass

    return ret_name


def list_to_transformation_matrix(values, fbb):
    """Puts a list of values into an FBB TransformationMatrix."""

    TransformationMatrix.TransformationMatrixStartDataVector(fbb, len(values))
    for n in reversed(values):
        fbb.PrependFloat32(n)
    list_offset = fbb.EndVector()

    TransformationMatrix.TransformationMatrixStart(fbb)
    TransformationMatrix.TransformationMatrixAddData(fbb, list_offset)
    return TransformationMatrix.TransformationMatrixEnd(fbb)


def load_camera_definitions(year):
    """
    CAMERA DEFINITIONS
    We only load in cameras that have a calibration file
    These are stored in y<year>/vision/calib_files

    Or better yet, use //y2020/vision:calibration to calibrate the camera
    using a Charuco target board
    """

    dir_name = bazel_name_fix("calib_files", year)
    if dir_name is not None:
        glog.debug("Searching for calibration files in " + dir_name)
    else:
        glog.fatal("Failed to find calib_files directory")

    camera_calib_list = []
    for filename in sorted(os.listdir(dir_name)):
        glog.debug("Inspecting %s", filename)
        if ("cam-calib-int" in filename
                or 'calibration' in filename) and filename.endswith(".json"):
            # Extract intrinsics from file
            calib_file = open(dir_name + "/" + filename, 'r')
            calib_dict = json.loads(calib_file.read())
            camera_calib_list.append(calib_dict)
    return camera_calib_list


def generate_header(year):
    """
    Generates a header file with a CalibrationData() function
    returning the data as a binary flatbuffer.
    Expects command line argument: output header path
    Parameter year is a string, ex. "2023"
    """
    camera_calib_list = load_camera_definitions(year)

    output_path = sys.argv[1]
    glog.debug("Writing file to %s", output_path)

    fbb = flatbuffers.Builder(0)

    images_vector = []

    # Create camera calibration data
    camera_calibration_vector = []
    for camera_calib in camera_calib_list:
        fixed_extrinsics_list = camera_calib["fixed_extrinsics"]
        fixed_extrinsics_vector = list_to_transformation_matrix(
            fixed_extrinsics_list, fbb)

        turret_extrinsics_vector = None
        if "turret_extrinsics" in camera_calib:
            turret_extrinsics_list = camera_calib["turret_extrinsics"]
            turret_extrinsics_vector = list_to_transformation_matrix(
                turret_extrinsics_list, fbb)

        camera_int_list = camera_calib["intrinsics"]
        CameraCalibration.CameraCalibrationStartIntrinsicsVector(
            fbb, len(camera_int_list))
        for n in reversed(camera_int_list):
            fbb.PrependFloat32(n)
        intrinsics_vector = fbb.EndVector()

        dist_coeffs_list = camera_calib["dist_coeffs"]
        CameraCalibration.CameraCalibrationStartDistCoeffsVector(
            fbb, len(dist_coeffs_list))
        for n in reversed(dist_coeffs_list):
            fbb.PrependFloat32(n)
        dist_coeffs_vector = fbb.EndVector()

        node_name_offset = fbb.CreateString(camera_calib["node_name"])
        CameraCalibration.CameraCalibrationStart(fbb)
        CameraCalibration.CameraCalibrationAddNodeName(fbb, node_name_offset)
        CameraCalibration.CameraCalibrationAddTeamNumber(
            fbb, camera_calib["team_number"])
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

    # "year" will get replaced by the variable
    output_prefix = [
        '#ifndef Y{year}_VISION_CALIBRATION_DATA_H_',
        '#define Y{year}_VISION_CALIBRATION_DATA_H_',
        '#include <stdint.h>',
        '#include "absl/types/span.h"',
        'namespace y{year} {{',
        'namespace vision {{',
        'inline absl::Span<const uint8_t> CalibrationData() {{',
    ]
    output_suffix = [
        '  return absl::Span<const uint8_t>(reinterpret_cast<const uint8_t *>(kData), sizeof(kData));',
        '}}',
        '}}  // namespace vision',
        '}}  // namespace y{year}',
        '#endif  // Y{year}_VISION_CALIBRATION_DATA_H_',
    ]

    # Write out the header file
    with open(output_path, 'wb') as output:
        for line in output_prefix:
            output.write(line.format(year=year).encode("utf-8"))
            output.write(b'\n')
        output.write(b'alignas(64) static constexpr char kData[] = "')
        for byte in fbb.Output():
            output.write(b'\\x' + (b'%x' % byte).zfill(2))
        output.write(b'";\n')
        for line in output_suffix:
            output.write(line.format(year=year).encode("utf-8"))
            output.write(b'\n')
