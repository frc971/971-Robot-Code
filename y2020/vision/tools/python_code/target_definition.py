import cv2
import json
import math
import numpy as np

import camera_definition
import define_training_data as dtd
import train_and_match as tam

VISUALIZE_KEYPOINTS = True
VISUALIZE_POLYGONS = True


class TargetData:
    def __init__(self, filename):
        self.image_filename = filename
        # Load an image (will come in as a 1-element list)
        self.image = tam.load_images([filename])[0]
        self.polygon_list = []
        self.polygon_list_3d = []
        self.reprojection_map_list = []
        self.keypoint_list = []
        self.keypoint_list_3d = None  # numpy array of 3D points
        self.descriptor_list = []

    def extract_features(self, feature_extractor):
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
        point_list_3d = np.asarray(
            [(0., 0., 0.) for kp in keypoint_list]).reshape(-1, 3)
        # Iterate through our polygons
        for poly_ind in range(len(self.polygon_list)):
            # Filter and project points for each polygon in the list
            filtered_keypoints, _, _, _, keep_list = dtd.filter_keypoints_by_polygons(
                keypoint_list, None, [self.polygon_list[poly_ind]])
            print("Filtering kept %d of %d features" % (len(keep_list),
                                                        len(keypoint_list)))
            filtered_point_array = np.asarray(
                [(keypoint.pt[0], keypoint.pt[1])
                 for keypoint in filtered_keypoints]).reshape(-1, 2)
            filtered_point_list_3d = dtd.compute_3d_points(
                filtered_point_array, self.reprojection_map_list[poly_ind])
            for i in range(len(keep_list)):
                point_list_3d[keep_list[i]] = filtered_point_list_3d[i]

        return point_list_3d

    # Save out the training data-- haven't implemented this
    def save_training_data(self):
        if (len(self.keypoint_list) != len(self.descriptor_list)
                or len(self.keypoint_list) != len(self.keypoint_list_3d)):
            print("Big problem-- lists don't match in size: %d, %d, %d" %
                  (len(self.keypoint_list), len(self.descriptor_list),
                   len(self.keypoint_list_3d)))

        print("(Would like to) Save target with %d keypoints" % len(
            self.keypoint_list))


# TARGET DEFINITIONS

# Some general info about our field and targets
# Assume camera centered on target at 1 m above ground and distance of 4.85m
camera_height = 1.
total_target_height = 3.10
field_length = 15.98
power_port_offset_y = 1.67
power_port_width = 1.22
bottom_wing_height = 1.88
wing_width = 1.83

# Pick the target center location at halfway between top and bottom of the top panel
target_center_height = (total_target_height + bottom_wing_height) / 2.

# TODO: Still need to figure out what this angle actually is
wing_angle = 20. * math.pi / 180.

# Start at lower left corner, and work around clockwise
# These are taken by manually finding the points in gimp for this image
main_panel_polygon_points_2d = [(451, 679), (451, 304), (100, 302), (451, 74),
                                (689, 74), (689, 302), (689, 679)]
# These are "virtual" 3D points based on the expected geometry
main_panel_polygon_points_3d = [
    (field_length, power_port_width / 2. - power_port_offset_y,
     0.), (field_length, power_port_width / 2. - power_port_offset_y,
           bottom_wing_height),
    (field_length, power_port_width / 2. - power_port_offset_y + wing_width,
     bottom_wing_height),
    (field_length, power_port_width / 2. - power_port_offset_y,
     total_target_height), (field_length,
                            -power_port_width / 2. - power_port_offset_y,
                            total_target_height),
    (field_length, -power_port_width / 2. - power_port_offset_y,
     bottom_wing_height), (field_length,
                           -power_port_width / 2. - power_port_offset_y, 0.)
]

wing_panel_polygon_points_2d = [(689, 74), (1022, 302), (689, 302)]
# These are "virtual" 3D points based on the expected geometry
wing_panel_polygon_points_3d = [
    (field_length, -power_port_width / 2. - power_port_offset_y,
     total_target_height),
    (field_length - wing_width * math.sin(wing_angle), -power_port_width / 2. -
     power_port_offset_y - wing_width * math.cos(wing_angle),
     bottom_wing_height), (field_length,
                           -power_port_width / 2. - power_port_offset_y,
                           bottom_wing_height)
]

# Populate the red power port
ideal_power_port_red = TargetData('test_images/train_power_port_red.png')

ideal_power_port_red.polygon_list.append(main_panel_polygon_points_2d)
ideal_power_port_red.polygon_list_3d.append(main_panel_polygon_points_3d)

ideal_power_port_red.polygon_list.append(wing_panel_polygon_points_2d)
ideal_power_port_red.polygon_list_3d.append(wing_panel_polygon_points_3d)

ideal_target_list = []
ideal_target_list.append(ideal_power_port_red)

# Create feature extractor
feature_extractor = tam.load_feature_extractor()

# Use webcam parameters for now
camera_params = camera_definition.web_cam_params
for ideal_target in ideal_target_list:
    print("Preparing target for image %s" % ideal_target.image_filename)
    ideal_target.extract_features(feature_extractor)
    ideal_target.filter_keypoints_by_polygons()
    ideal_target.compute_reprojection_maps()
    ideal_target.keypoint_list_3d = ideal_target.project_keypoint_to_3d_by_polygon(
        ideal_target.keypoint_list)

    if VISUALIZE_POLYGONS:
        # For each polygon, show the 2D points and the reprojection for 3D
        # of the polygon definition
        for polygon_2d, polygon_3d in zip(ideal_target.polygon_list,
                                          ideal_target.polygon_list_3d):
            ideal_pts_tmp = np.asarray(polygon_2d).reshape(-1, 2)
            ideal_pts_3d_tmp = np.asarray(polygon_3d).reshape(-1, 3)
            # We can only compute pose if we have at least 4 points
            # Only matters for reprojection for visualization
            # Keeping this code here, since it's helpful when testing
            if (len(polygon_2d) >= 4):
                img_copy = dtd.draw_polygon(ideal_target.image.copy(), polygon_2d, (0,255,0), True)
                dtd.visualize_reprojections(img_copy, ideal_pts_tmp, ideal_pts_3d_tmp, camera_params.camera_int.camera_matrix, camera_params.camera_int.distortion_coeffs)

    if VISUALIZE_KEYPOINTS:
        # For each polygon, show which keypoints (2D and 3D manual) were kept
        for polygon in ideal_target.polygon_list:
            img_copy = ideal_target.image.copy()
            kp_in_poly2d = []
            kp_in_poly3d = []
            for kp, kp_3d in zip(ideal_target.keypoint_list,
                                 ideal_target.keypoint_list_3d):
                if dtd.point_in_polygons((kp.pt[0], kp.pt[1]), [polygon]):
                    kp_in_poly2d.append((kp.pt[0], kp.pt[1]))
                    kp_in_poly3d.append(kp_3d)

            img_copy = dtd.draw_polygon(img_copy, polygon, (0,255,0), True)
            dtd.visualize_reprojections(
                img_copy,
                np.asarray(kp_in_poly2d).reshape(-1, 2),
                np.asarray(kp_in_poly3d).reshape(
                    -1, 3), camera_params.camera_int.camera_matrix,
                camera_params.camera_int.distortion_coeffs)

### TODO: Add code to do manual point selection
AUTO_PROJECTION = True
training_target_list = []
if AUTO_PROJECTION:
    print("Auto projection of training keypoints to 3D using ideal images")
    # Match the captured training image against the "ideal" training image
    # and use those matches to pin down the 3D locations of the keypoints

    training_power_port_red = TargetData(
        'test_images/train_power_port_red_webcam.png')

    training_target_list.append(training_power_port_red)

    for target_ind in range(len(training_target_list)):
        # Assumes we have 1 ideal view for each training target
        training_target = training_target_list[target_ind]
        ideal_target = ideal_target_list[target_ind]

        print("Preparing target for image %s" % training_target.image_filename)
        # Extract keypoints and descriptors for model
        training_target.extract_features(feature_extractor)

        # Create matcher that we'll use to match with ideal
        matcher = tam.train_matcher([training_target.descriptor_list])

        matches_list = tam.compute_matches(matcher,
                                           [training_target.descriptor_list],
                                           [ideal_target.descriptor_list])

        homography_list, matches_mask_list = tam.compute_homographies(
            [training_target.keypoint_list], [ideal_target.keypoint_list],
            matches_list)

        for polygon in ideal_target.polygon_list:
            ideal_pts_2d = np.asarray(np.float32(polygon)).reshape(-1, 1, 2)
            H_inv = np.linalg.inv(homography_list[0])
            # We use the ideal target's polygons to define the polygons on
            # the training target
            transformed_polygon = cv2.perspectiveTransform(ideal_pts_2d, H_inv)
            training_target.polygon_list.append(transformed_polygon)

            # Verify that this looks right
            if VISUALIZE_KEYPOINTS:
                pts = transformed_polygon.astype(int).reshape(-1, 2)
                image = dtd.draw_polygon(training_target.image.copy(), pts,
                                         (255, 0, 0), True)
                cv2.imshow("image", image)
                cv2.waitKey(0)

        print("Started with %d keypoints" % len(training_target.keypoint_list))

        training_target.keypoint_list, training_target.descriptor_list, rejected_keypoint_list, rejected_descriptor_list, _ = dtd.filter_keypoints_by_polygons(
            training_target.keypoint_list, training_target.descriptor_list,
            training_target.polygon_list)
        print("After filtering by polygons, had %d keypoints" % len(
            training_target.keypoint_list))
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

        training_target.keypoint_list_3d = np.asarray(
            training_3d_list).reshape(-1, 1, 3)

        if VISUALIZE_KEYPOINTS:
            # Sanity check these:
            img_copy = training_target.image.copy()
            kp_tmp = np.asarray([
                (kp.pt[0], kp.pt[1]) for kp in training_target.keypoint_list
            ]).reshape(-1, 2)
            dtd.visualize_reprojections(
                img_copy, kp_tmp, training_target.keypoint_list_3d,
                camera_params.camera_int.camera_matrix,
                camera_params.camera_int.distortion_coeffs)

        training_target.save_training_data()

y2020_target_list = training_target_list
