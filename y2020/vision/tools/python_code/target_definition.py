import argparse
import cv2
# TODO<Jim>: Add gflags for handling command-line flags
import glog
import json
import math
import numpy as np

import camera_definition
import define_training_data as dtd
import train_and_match as tam

# TODO<Jim>: Allow command-line setting of logging level
glog.setLevel("INFO")
global VISUALIZE_KEYPOINTS, AUTO_PROJECTION
VISUALIZE_KEYPOINTS = False
AUTO_PROJECTION = False

# For now, just have a 12 pixel radius, based on captured (taped) training image
target_radius_default = 12.


class TargetData:
    def __init__(self, filename=None):
        if filename:
            self.image_filename = dtd.bazel_name_fix(filename)
            # Load an image (will come in as a 1-element list)
            self.image = tam.load_images([self.image_filename])[0]
        self.polygon_list = []
        self.polygon_list_3d = []
        self.reprojection_map_list = []
        self.keypoint_list = []
        self.keypoint_list_3d = None  # numpy array of 3D points
        self.descriptor_list = []
        self.target_rotation = None
        self.target_position = None
        self.target_point_2d = None
        self.target_radius = target_radius_default

    @classmethod
    def from_json(cls, filename):
        data = None
        with open(dtd.bazel_name_fix(filename), 'r') as fd:
            data = json.load(fd)
        # Convert lists that should be np arrays
        data["target_rotation"] = np.array(data["target_rotation"])
        data["target_position"] = np.array(data["target_position"])
        if "target_point_2d" in data:
            data["target_point_2d"] = np.array(data["target_point_2d"])
        # Load an image (will come in as a 1-element list)
        if not data["image_filename"].startswith("org_frc971"):
            data["image_filename"] = dtd.bazel_name_fix(data["image_filename"])
        data["image"] = tam.load_images([data["image_filename"]])[0]
        data["reprojection_map_list"] = []
        data["keypoint_list"] = []
        data["keypoint_list_3d"] = None  # numpy array of 3D points
        data["descriptor_list"] = []
        td = cls()
        td.__dict__ = data
        return td

    @staticmethod
    def from_jsons(*filenames):
        tds = []
        for filename in filenames:
            tds.append(TargetData.from_json(filename))
        return tds

    def write_to_json(self, filename):
        with open(filename, 'w') as fd:
            # Convert np arrays to lists, which are json serializable
            data = self.__dict__.copy()
            data["target_rotation"] = self.target_rotation.tolist()
            data["target_position"] = self.target_position.tolist()
            if self.target_point_2d is not None:
                data["target_point_2d"] = self.target_point_2d.tolist()
            # Don't store the large image since we have its filename,
            # and the data that is always blank at this point
            for field in [
                    "image", "reprojection_map_list", "keypoint_list",
                    "keypoint_list_3d", "descriptor_list"
            ]:
                data.pop(field)
            json.dump(data, fd)

    def extract_features(self, feature_extractor=None):
        if feature_extractor is None:
            feature_extractor = tam.load_feature_extractor()

        kp_lists, desc_lists = tam.detect_and_compute(feature_extractor,
                                                      [self.image])
        self.keypoint_list = kp_lists[0]
        self.descriptor_list = desc_lists[0]

    def filter_keypoints_by_polygons(self):
        self.keypoint_list, self.descriptor_list, _, _, _ = dtd.filter_keypoints_by_polygons(
            self.keypoint_list, self.descriptor_list, self.polygon_list)

    def compute_reprojection_maps(self):
        for poly_ind in range(len(self.polygon_list)):
            reprojection_map = dtd.compute_reprojection_map(
                self.polygon_list[poly_ind], self.polygon_list_3d[poly_ind])
            self.reprojection_map_list.append(reprojection_map)

    def project_keypoint_to_3d_by_polygon(self, keypoint_list):
        # Create dummy array of correct size that we can put values in
        point_list_3d = np.asarray([(0., 0., 0.)
                                    for kp in keypoint_list]).reshape(-1, 3)
        # Iterate through our polygons
        for poly_ind in range(len(self.polygon_list)):
            # Filter and project points for each polygon in the list
            filtered_keypoints, _, _, _, keep_list = dtd.filter_keypoints_by_polygons(
                keypoint_list, None, [self.polygon_list[poly_ind]])
            glog.debug("Filtering kept %d of %d features" %
                       (len(keep_list), len(keypoint_list)))
            filtered_point_array = np.asarray([
                (keypoint.pt[0], keypoint.pt[1])
                for keypoint in filtered_keypoints
            ]).reshape(-1, 2)
            filtered_point_list_3d = dtd.compute_3d_points(
                filtered_point_array, self.reprojection_map_list[poly_ind])
            for i in range(len(keep_list)):
                point_list_3d[keep_list[i]] = filtered_point_list_3d[i]

        return point_list_3d


def load_training_data():
    ######################################################################
    # DEFINE the targets here.  Generate lists of ideal and training
    # targets based on the json files in target_definitions
    ######################################################################
    ideal_target_list = TargetData.from_jsons(
        "target_definitions/ideal_power_port_taped.json",
        "target_definitions/ideal_power_port_red.json",
        "target_definitions/ideal_power_port_blue.json")
    training_target_list = TargetData.from_jsons(
        "target_definitions/training_target_power_port_taped.json",
        "target_definitions/training_target_power_port_red.json",
        "target_definitions/training_target_power_port_blue.json")
    return ideal_target_list, training_target_list


def compute_target_definition(camera_params=None):
    if camera_params is None:
        camera_params = camera_definition.load_pi1_camera_params()

    ideal_target_list, training_target_list = load_training_data()

    # Create feature extractor
    feature_extractor = tam.load_feature_extractor()

    # Extract features from ideal targets if we are going to auto-project,
    # otherwise get them directly from the training targets.
    for target in (ideal_target_list
                   if AUTO_PROJECTION else training_target_list):
        glog.debug("\nPreparing target for image %s" % target.image_filename)
        target.extract_features(feature_extractor)
        target.filter_keypoints_by_polygons()
        target.compute_reprojection_maps()
        target.keypoint_list_3d = target.project_keypoint_to_3d_by_polygon(
            target.keypoint_list)

        if VISUALIZE_KEYPOINTS:
            for i in range(len(target.polygon_list)):
                pts_tmp = np.asarray(target.polygon_list[i]).reshape(-1, 2)
                pts_3d_tmp = np.asarray(target.polygon_list_3d[i]).reshape(
                    -1, 3)
                # We can only compute pose if we have at least 4 points
                # Only matters for reprojection for visualization
                # Keeping this code here, since it's helpful when testing
                if (len(target.polygon_list[i]) >= 4):
                    img_copy = dtd.draw_polygon(target.image.copy(),
                                                target.polygon_list[i],
                                                (0, 255, 0), True)
                    dtd.visualize_reprojections(
                        img_copy, pts_tmp, pts_3d_tmp,
                        camera_params.camera_int.camera_matrix,
                        camera_params.camera_int.dist_coeffs)

            for polygon in target.polygon_list:
                img_copy = target.image.copy()
                kp_in_poly2d = []
                kp_in_poly3d = []
                for kp, kp_3d in zip(target.keypoint_list,
                                     target.keypoint_list_3d):
                    if dtd.point_in_polygons((kp.pt[0], kp.pt[1]), [polygon]):
                        kp_in_poly2d.append((kp.pt[0], kp.pt[1]))
                        kp_in_poly3d.append(kp_3d)

                dtd.visualize_reprojections(
                    img_copy,
                    np.asarray(kp_in_poly2d).reshape(-1, 2),
                    np.asarray(kp_in_poly3d).reshape(-1, 3),
                    camera_params.camera_int.camera_matrix,
                    camera_params.camera_int.dist_coeffs)

    ###############
    ### Compute 3D points on actual training images
    ###############
    if AUTO_PROJECTION:
        glog.debug(
            "Doing auto projection of training keypoints to 3D using ideal images"
        )
        # Match the captured training image against the "ideal" training image
        # and use those matches to pin down the 3D locations of the keypoints

        for target_ind in range(len(training_target_list)):
            # Assumes we have 1 ideal view for each training target
            training_target = training_target_list[target_ind]
            glog.check_eq(
                len(training_target.polygon_list), 0,
                "Expected training target with emty polygon list for auto projection"
            )
            ideal_target = ideal_target_list[target_ind]

            glog.debug("\nPreparing target for image %s" %
                       training_target.image_filename)
            # Extract keypoints and descriptors for model
            training_target.extract_features(feature_extractor)

            # Create matcher that we'll use to match with ideal
            matcher = tam.train_matcher([training_target.descriptor_list])

            matches_list = tam.compute_matches(
                matcher, [training_target.descriptor_list],
                [ideal_target.descriptor_list])

            homography_list, matches_mask_list = tam.compute_homographies(
                [training_target.keypoint_list], [ideal_target.keypoint_list],
                matches_list)

            # Compute H_inv, since this maps points in ideal target to
            # points in the training target
            H_inv = np.linalg.inv(homography_list[0])

            for polygon in ideal_target.polygon_list:
                ideal_pts_2d = np.asarray(np.float32(polygon)).reshape(
                    -1, 1, 2)
                # We use the ideal target's polygons to define the polygons on
                # the training target
                transformed_polygon = cv2.perspectiveTransform(
                    ideal_pts_2d, H_inv)
                training_target.polygon_list.append(transformed_polygon)

            # Also project the target point from the ideal to training image
            training_target_point_2d = cv2.perspectiveTransform(
                np.asarray(ideal_target.target_point_2d).reshape(-1, 1, 2),
                H_inv)
            training_target.target_point_2d = training_target_point_2d.reshape(
                -1, 1, 2)

            glog.debug("Started with %d keypoints" %
                       len(training_target.keypoint_list))

            training_target.keypoint_list, training_target.descriptor_list, rejected_keypoint_list, rejected_descriptor_list, _ = dtd.filter_keypoints_by_polygons(
                training_target.keypoint_list, training_target.descriptor_list,
                training_target.polygon_list)
            glog.debug("After filtering by polygons, had %d keypoints" %
                       len(training_target.keypoint_list))
            if VISUALIZE_KEYPOINTS:
                tam.show_keypoints(training_target.image,
                                   training_target.keypoint_list)

            # Now comes the fun part
            # Go through all my training keypoints to define 3D location using ideal
            training_3d_list = []
            for kp_ind in range(len(training_target.keypoint_list)):
                # We're going to look for the first time this keypoint is in a polygon
                found_3d_loc = False
                # First, is it in the correct polygon
                kp_loc = (training_target.keypoint_list[kp_ind].pt[0],
                          training_target.keypoint_list[kp_ind].pt[1])
                for poly_ind in range(len(training_target.polygon_list)):
                    if dtd.point_in_polygons(
                            kp_loc, [training_target.polygon_list[poly_ind]
                                     ]) and not found_3d_loc:
                        found_3d_loc = True
                        # If so, transform keypoint location to ideal using homography, and compute 3D
                        kp_loc_array = np.asarray(np.float32(kp_loc)).reshape(
                            -1, 1, 2)
                        training_2d_in_ideal = cv2.perspectiveTransform(
                            kp_loc_array, homography_list[0])
                        # Get 3D from this 2D point in ideal image
                        training_3d_pt = dtd.compute_3d_points(
                            training_2d_in_ideal,
                            ideal_target.reprojection_map_list[poly_ind])
                        training_3d_list.append(training_3d_pt)

            training_target.keypoint_list_3d = np.asarray(training_3d_list)

            if VISUALIZE_KEYPOINTS:
                # Sanity check these:
                img_copy = training_target.image.copy()
                for polygon in training_target.polygon_list:
                    pts = polygon.astype(int).reshape(-1, 2)
                    img_copy = dtd.draw_polygon(img_copy, pts, (255, 0, 0),
                                                True)
                kp_tmp = np.asarray([(kp.pt[0], kp.pt[1])
                                     for kp in training_target.keypoint_list
                                     ]).reshape(-1, 2)
                dtd.visualize_reprojections(
                    img_copy, kp_tmp, training_target.keypoint_list_3d,
                    camera_params.camera_int.camera_matrix,
                    camera_params.camera_int.dist_coeffs)

    y2020_target_list = training_target_list
    return y2020_target_list


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("--visualize",
                    help="Whether to visualize the results",
                    default=False,
                    action='store_true')
    ap.add_argument(
        "--auto_projection",
        help=
        "Whether to auto-project 2d points from the ideal targets to the training ones",
        default=False,
        action="store_true")
    args = vars(ap.parse_args())

    VISUALIZE_KEYPOINTS = args["visualize"]
    if VISUALIZE_KEYPOINTS:
        glog.info("Visualizing results")

    AUTO_PROJECTION = args["auto_projection"]
    if AUTO_PROJECTION:
        glog.info("Auto projecting points")

    # Computes target defintion using default (pi1) camera params
    compute_target_definition()
