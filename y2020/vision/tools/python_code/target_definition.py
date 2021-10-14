import argparse
import cv2
# TODO<Jim>: Add gflags for handling command-line flags
import glog
import math
import numpy as np

import camera_definition
import define_training_data as dtd
import train_and_match as tam

# TODO<Jim>: Allow command-line setting of logging level
glog.setLevel("INFO")
global VISUALIZE_KEYPOINTS
VISUALIZE_KEYPOINTS = False

# For now, just have a 12 pixel radius, based on captured (taped) training image
target_radius_default = 12.


class TargetData:
    def __init__(self, filename):
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
    ############################################################
    # TARGET DEFINITIONS
    ############################################################

    ideal_target_list = []
    training_target_list = []

    # Using coordinate system as defined in sift.fbs:
    # field origin (0, 0, 0) at floor height at center of field
    # Robot orientation with x-axis pointing towards RED ALLIANCE player station
    # y-axis to left and z-axis up (right-hand coordinate system)
    # Thus, the red power port target location will be at (-15.98/2,1.67,0),
    # with orientation (facing out of the field) of M_PI

    # Field constants
    field_length = 15.983
    power_port_total_height = 3.10
    power_port_edge_y = 1.089
    power_port_width = 1.225
    power_port_bottom_wing_height = 1.828
    power_port_wing_width = 1.83
    loading_bay_edge_y = 0.784
    loading_bay_width = 1.524
    loading_bay_height = 0.933
    # Wing angle on actual field target
    # wing_angle = 20. * math.pi / 180.
    ### NOTE: Setting wing angle to zero to match current FRC971 target
    wing_angle = 0

    # Pick the target center location at halfway between top and bottom of the top panel
    power_port_target_height = (power_port_total_height +
                                power_port_bottom_wing_height) / 2.

    ### Taped up FRC target definition
    inch_to_meter = 0.0254
    c_power_port_total_height = (79.5 + 39.5) * inch_to_meter
    c_power_port_edge_y = 1.089
    c_power_port_width = 4.0 * 12 * inch_to_meter
    c_power_port_bottom_wing_height = 79.5 * inch_to_meter
    c_power_port_wing_width = 47.5 * inch_to_meter
    c_power_port_white_marker_z = (79.5 - 19.5) * inch_to_meter

    # Pick the target center location at halfway between top and bottom of the top panel
    c_power_port_target_height = (power_port_total_height +
                                  power_port_bottom_wing_height) / 2.

    ###
    ### Taped power port
    ###

    # Create the reference "ideal" image.
    # NOTE: Since we don't have an "ideal" (e.g., graphic) version, we're
    # just using the same image as the training image.
    ideal_power_port_taped = TargetData(
        'test_images/partial_971_power_port_red_daytime.png')

    # Start at lower left corner, and work around clockwise
    # These are taken by manually finding the points in gimp for this image
    power_port_taped_main_panel_polygon_points_2d = [(198, 473), (203, 154),
                                                     (23, 156), (204, 19),
                                                     (374, 16), (381, 152),
                                                     (397, 467)]

    # These are "virtual" 3D points based on the expected geometry
    power_port_taped_main_panel_polygon_points_3d = [
        (-field_length / 2., c_power_port_edge_y, 0.),
        (-field_length / 2., c_power_port_edge_y,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y - c_power_port_wing_width,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y, c_power_port_total_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width,
         c_power_port_total_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width, 0.)
    ]

    power_port_taped_wing_panel_polygon_points_2d = [(312, 99), (438, 191),
                                                     (315, 195)]

    # These are "virtual" 3D points based on the expected geometry
    power_port_taped_wing_panel_polygon_points_3d = [
        (field_length / 2., -power_port_edge_y - power_port_width,
         power_port_total_height),
        (field_length / 2. - power_port_wing_width * math.sin(wing_angle),
         -power_port_edge_y - power_port_width -
         power_port_wing_width * math.cos(wing_angle),
         power_port_bottom_wing_height),
        (field_length / 2., -power_port_edge_y - power_port_width,
         power_port_bottom_wing_height)
    ]

    # Populate the taped power port
    ideal_power_port_taped.polygon_list.append(
        power_port_taped_main_panel_polygon_points_2d)
    ideal_power_port_taped.polygon_list_3d.append(
        power_port_taped_main_panel_polygon_points_3d)
    # Including the wing panel
    #ideal_power_port_taped.polygon_list.append(
    #    power_port_taped_wing_panel_polygon_points_2d)
    #ideal_power_port_taped.polygon_list_3d.append(
    #    power_port_taped_wing_panel_polygon_points_3d)

    # Location of target.  Rotation is pointing in -x direction
    ideal_power_port_taped.target_rotation = -np.identity(3, np.double)
    ideal_power_port_taped.target_position = np.array([
        -field_length / 2., c_power_port_edge_y + c_power_port_width / 2.,
        c_power_port_target_height
    ])
    ideal_power_port_taped.target_point_2d = np.float32([[290, 87]]).reshape(
        -1, 1, 2)  # partial_971_power_port_red_daytime.png

    training_target_power_port_taped = TargetData(
        'test_images/partial_971_power_port_red_daytime.png')
    training_target_power_port_taped.target_rotation = ideal_power_port_taped.target_rotation
    training_target_power_port_taped.target_position = ideal_power_port_taped.target_position
    training_target_power_port_taped.target_radius = target_radius_default

    ###
    ### Taped power port -- far away image
    ###

    # Create the reference "ideal" image.
    # NOTE: Since we don't have an "ideal" (e.g., graphic) version, we're
    # just using the same image as the training image.
    ideal_power_port_taped_far = TargetData(
        'test_images/partial_971_power_port_red_daytime_far.png')

    # Start at lower left corner, and work around clockwise
    # These are taken by manually finding the points in gimp for this image
    power_port_taped_far_main_panel_polygon_points_2d = [
        (259, 363), (255, 230), (178, 234), (255, 169), (329, 164), (334, 225),
        (341, 361)
    ]

    # These are "virtual" 3D points based on the expected geometry
    power_port_taped_far_main_panel_polygon_points_3d = [
        (-field_length / 2., c_power_port_edge_y, 0.),
        (-field_length / 2., c_power_port_edge_y,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y - c_power_port_wing_width,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y, c_power_port_total_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width,
         c_power_port_total_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width, 0.)
    ]

    # Populate the taped power port
    ideal_power_port_taped_far.polygon_list.append(
        power_port_taped_far_main_panel_polygon_points_2d)
    ideal_power_port_taped_far.polygon_list_3d.append(
        power_port_taped_far_main_panel_polygon_points_3d)

    # Location of target.  Rotation is pointing in -x direction
    ideal_power_port_taped_far.target_rotation = -np.identity(3, np.double)
    ideal_power_port_taped_far.target_position = np.array([
        -field_length / 2., c_power_port_edge_y + c_power_port_width / 2.,
        c_power_port_target_height
    ])
    ideal_power_port_taped_far.target_point_2d = np.float32(
        [[294, 198]]).reshape(-1, 1,
                              2)  # partial_971_power_port_red_daytime.png

    training_target_power_port_taped_far = TargetData(
        'test_images/partial_971_power_port_red_daytime_far.png')
    training_target_power_port_taped_far.target_rotation = ideal_power_port_taped_far.target_rotation
    training_target_power_port_taped_far.target_position = ideal_power_port_taped_far.target_position
    training_target_power_port_taped_far.target_radius = target_radius_default

    ###
    ### Taped power port night image
    ###

    # Create the reference "ideal" image.
    # NOTE: Since we don't have an "ideal" (e.g., graphic) version, we're
    # just using the same image as the training image.
    ideal_power_port_taped_night = TargetData(
        'test_images/partial_971_power_port_red_nighttime.png')

    # Start at lower left corner, and work around clockwise
    # These are taken by manually finding the points in gimp for this image
    power_port_taped_night_main_panel_polygon_points_2d = [
        (217, 425), (215, 187), (78, 189), (212, 80), (344, 74), (347, 180),
        (370, 421)
    ]

    # These are "virtual" 3D points based on the expected geometry
    power_port_taped_night_main_panel_polygon_points_3d = [
        (-field_length / 2., c_power_port_edge_y, 0.),
        (-field_length / 2., c_power_port_edge_y,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y - c_power_port_wing_width,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y, c_power_port_total_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width,
         c_power_port_total_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width,
         c_power_port_bottom_wing_height),
        (-field_length / 2., c_power_port_edge_y + c_power_port_width, 0.)
    ]

    # Populate the taped power port
    ideal_power_port_taped_night.polygon_list.append(
        power_port_taped_night_main_panel_polygon_points_2d)
    ideal_power_port_taped_night.polygon_list_3d.append(
        power_port_taped_night_main_panel_polygon_points_3d)

    # Location of target.  Rotation is pointing in -x direction
    ideal_power_port_taped_night.target_rotation = -np.identity(3, np.double)
    ideal_power_port_taped_night.target_position = np.array([
        -field_length / 2., c_power_port_edge_y + c_power_port_width / 2.,
        c_power_port_target_height
    ])
    ideal_power_port_taped_night.target_point_2d = np.float32(
        [[282, 132]]).reshape(-1, 1, 2)  # partial_971_power_port_red_night.png

    training_target_power_port_taped_night = TargetData(
        'test_images/partial_971_power_port_red_nighttime.png')
    training_target_power_port_taped_night.target_rotation = ideal_power_port_taped_night.target_rotation
    training_target_power_port_taped_night.target_position = ideal_power_port_taped_night.target_position
    training_target_power_port_taped_night.target_radius = target_radius_default

    ###
    ### Red Power Port
    ###

    # Create the reference "ideal" image
    ideal_power_port_red = TargetData('test_images/ideal_power_port_red.png')

    # Start at lower left corner, and work around clockwise
    # These are taken by manually finding the points in gimp for this image
    power_port_red_main_panel_polygon_points_2d = [(451, 679), (451, 304),
                                                   (100, 302), (451, 74),
                                                   (689, 74), (689, 302),
                                                   (689, 679)]

    # These are "virtual" 3D points based on the expected geometry
    power_port_red_main_panel_polygon_points_3d = [
        (-field_length / 2., power_port_edge_y, 0.),
        (-field_length / 2., power_port_edge_y, power_port_bottom_wing_height),
        (-field_length / 2., power_port_edge_y - power_port_wing_width,
         power_port_bottom_wing_height),
        (-field_length / 2., power_port_edge_y, power_port_total_height),
        (-field_length / 2., power_port_edge_y + power_port_width,
         power_port_total_height),
        (-field_length / 2., power_port_edge_y + power_port_width,
         power_port_bottom_wing_height),
        (-field_length / 2., power_port_edge_y + power_port_width, 0.)
    ]

    power_port_red_wing_panel_polygon_points_2d = [(689, 74), (1022, 302),
                                                   (689, 302)]
    # These are "virtual" 3D points based on the expected geometry
    power_port_red_wing_panel_polygon_points_3d = [
        (-field_length / 2., power_port_edge_y + power_port_width,
         power_port_total_height),
        (-field_length / 2. + power_port_wing_width * math.sin(wing_angle),
         power_port_edge_y + power_port_width +
         power_port_wing_width * math.cos(wing_angle),
         power_port_bottom_wing_height),
        (-field_length / 2., power_port_edge_y + power_port_width,
         power_port_bottom_wing_height)
    ]

    # Populate the red power port
    ideal_power_port_red.polygon_list.append(
        power_port_red_main_panel_polygon_points_2d)
    ideal_power_port_red.polygon_list_3d.append(
        power_port_red_main_panel_polygon_points_3d)
    # NOTE: We are currently not using the wing, since our actual targets are all planar

    # Define the pose of the target
    # Location is on the ground, at the center of the target
    # Orientation is with "x" pointing out of the field, and "z" up
    # This way, if robot is facing target directly, the x-axes are aligned
    # and the skew to target is zero
    ideal_power_port_red.target_rotation = -np.identity(3, np.double)
    ideal_power_port_red.target_rotation[2][2] = 1.
    ideal_power_port_red.target_position = np.array([
        -field_length / 2., power_port_edge_y + power_port_width / 2.,
        power_port_target_height
    ])

    # Target point on the image -- needs to match training image
    # These are manually captured by examining the images,
    # and entering the pixel values from the target center for each image.
    # These are currently only used for visualization of the target
    ideal_power_port_red.target_point_2d = np.float32([[570, 192]]).reshape(
        -1, 1, 2)  # ideal_power_port_red.png
    # np.float32([[305, 97]]).reshape(-1, 1, 2),  #train_power_port_red_webcam.png

    # And add the training image we'll actually use to the training list
    training_target_power_port_red = TargetData(
        'test_images/train_power_port_red.png')
    #'test_images/train_power_port_red_pi-7971-3.png')
    training_target_power_port_red.target_rotation = ideal_power_port_red.target_rotation
    training_target_power_port_red.target_position = ideal_power_port_red.target_position
    training_target_power_port_red.target_radius = target_radius_default

    ###
    ### Red Loading Bay
    ###

    ideal_loading_bay_red = TargetData('test_images/ideal_loading_bay_red.png')

    # Start at lower left corner, and work around clockwise
    # These are taken by manually finding the points in gimp for this image
    loading_bay_red_polygon_points_2d = [(42, 406), (42, 35), (651, 34),
                                         (651, 406)]

    # These are "virtual" 3D points based on the expected geometry
    loading_bay_red_polygon_points_3d = [
        (field_length / 2., loading_bay_edge_y + loading_bay_width, 0.),
        (field_length / 2., loading_bay_edge_y + loading_bay_width,
         loading_bay_height),
        (field_length / 2., loading_bay_edge_y, loading_bay_height),
        (field_length / 2., loading_bay_edge_y, 0.)
    ]

    ideal_loading_bay_red.polygon_list.append(
        loading_bay_red_polygon_points_2d)
    ideal_loading_bay_red.polygon_list_3d.append(
        loading_bay_red_polygon_points_3d)
    # Location of target
    ideal_loading_bay_red.target_rotation = np.identity(3, np.double)
    ideal_loading_bay_red.target_position = np.array([
        field_length / 2., loading_bay_edge_y + loading_bay_width / 2.,
        loading_bay_height / 2.
    ])
    ideal_loading_bay_red.target_point_2d = np.float32([[366, 236]]).reshape(
        -1, 1, 2)  # ideal_loading_bay_red.png

    training_target_loading_bay_red = TargetData(
        'test_images/train_loading_bay_red.png')
    training_target_loading_bay_red.target_rotation = ideal_loading_bay_red.target_rotation
    training_target_loading_bay_red.target_position = ideal_loading_bay_red.target_position
    training_target_loading_bay_red.target_radius = target_radius_default

    ###
    ### Blue Power Port
    ###

    ideal_power_port_blue = TargetData('test_images/ideal_power_port_blue.png')

    # Start at lower left corner, and work around clockwise
    # These are taken by manually finding the points in gimp for this image
    power_port_blue_main_panel_polygon_points_2d = [(438, 693), (438, 285),
                                                    (93, 285), (440, 50),
                                                    (692, 50), (692, 285),
                                                    (692, 693)]

    # These are "virtual" 3D points based on the expected geometry
    power_port_blue_main_panel_polygon_points_3d = [
        (field_length / 2., -power_port_edge_y, 0.),
        (field_length / 2., -power_port_edge_y, power_port_bottom_wing_height),
        (field_length / 2., -power_port_edge_y + power_port_wing_width,
         power_port_bottom_wing_height),
        (field_length / 2., -power_port_edge_y, power_port_total_height),
        (field_length / 2., -power_port_edge_y - power_port_width,
         power_port_total_height),
        (field_length / 2., -power_port_edge_y - power_port_width,
         power_port_bottom_wing_height),
        (field_length / 2., -power_port_edge_y - power_port_width, 0.)
    ]

    power_port_blue_wing_panel_polygon_points_2d = [(692, 50), (1047, 285),
                                                    (692, 285)]
    # These are "virtual" 3D points based on the expected geometry
    power_port_blue_wing_panel_polygon_points_3d = [
        (field_length / 2., -power_port_edge_y - power_port_width,
         power_port_total_height),
        (field_length / 2. - power_port_wing_width * math.sin(wing_angle),
         -power_port_edge_y - power_port_width -
         power_port_wing_width * math.cos(wing_angle),
         power_port_bottom_wing_height),
        (field_length / 2., -power_port_edge_y - power_port_width,
         power_port_bottom_wing_height)
    ]

    # Populate the blue power port
    ideal_power_port_blue.polygon_list.append(
        power_port_blue_main_panel_polygon_points_2d)
    ideal_power_port_blue.polygon_list_3d.append(
        power_port_blue_main_panel_polygon_points_3d)
    # Including the wing panel
    ideal_power_port_blue.polygon_list.append(
        power_port_blue_wing_panel_polygon_points_2d)
    ideal_power_port_blue.polygon_list_3d.append(
        power_port_blue_wing_panel_polygon_points_3d)

    # Location of target.  Rotation is pointing in -x direction
    ideal_power_port_blue.target_rotation = np.identity(3, np.double)
    ideal_power_port_blue.target_position = np.array([
        field_length / 2., -power_port_edge_y - power_port_width / 2.,
        power_port_target_height
    ])
    ideal_power_port_blue.target_point_2d = np.float32([[567, 180]]).reshape(
        -1, 1, 2)  # ideal_power_port_blue.png

    training_target_power_port_blue = TargetData(
        'test_images/train_power_port_blue.png')
    training_target_power_port_blue.target_rotation = ideal_power_port_blue.target_rotation
    training_target_power_port_blue.target_position = ideal_power_port_blue.target_position
    training_target_power_port_blue.target_radius = target_radius_default

    ###
    ### Blue Loading Bay
    ###

    ideal_loading_bay_blue = TargetData(
        'test_images/ideal_loading_bay_blue.png')

    # Start at lower left corner, and work around clockwise
    # These are taken by manually finding the points in gimp for this image
    loading_bay_blue_polygon_points_2d = [(7, 434), (7, 1), (729, 1),
                                          (729, 434)]

    # These are "virtual" 3D points based on the expected geometry
    loading_bay_blue_polygon_points_3d = [
        (-field_length / 2., -loading_bay_edge_y - loading_bay_width, 0.),
        (-field_length / 2., -loading_bay_edge_y - loading_bay_width,
         loading_bay_height),
        (-field_length / 2., -loading_bay_edge_y, loading_bay_height),
        (-field_length / 2., -loading_bay_edge_y, 0.)
    ]

    ideal_loading_bay_blue.polygon_list.append(
        loading_bay_blue_polygon_points_2d)
    ideal_loading_bay_blue.polygon_list_3d.append(
        loading_bay_blue_polygon_points_3d)

    # Location of target
    ideal_loading_bay_blue.target_rotation = -np.identity(3, np.double)
    ideal_loading_bay_blue.target_rotation[2][2] = 1.
    ideal_loading_bay_blue.target_position = np.array([
        -field_length / 2., -loading_bay_edge_y - loading_bay_width / 2.,
        loading_bay_height / 2.
    ])
    ideal_loading_bay_blue.target_point_2d = np.float32([[366, 236]]).reshape(
        -1, 1, 2)  # ideal_loading_bay_blue.png

    training_target_loading_bay_blue = TargetData(
        'test_images/train_loading_bay_blue.png')
    training_target_loading_bay_blue.target_rotation = ideal_loading_bay_blue.target_rotation
    training_target_loading_bay_blue.target_position = ideal_loading_bay_blue.target_position
    training_target_loading_bay_blue.target_radius = target_radius_default

    ######################################################################
    # DEFINE the targets here.  Generate lists of ideal and training
    # targets based on all the definitions above
    ######################################################################

    ### Taped power port
    glog.info("Adding hacked/taped up power port to the model list")
    ideal_target_list.append(ideal_power_port_taped)
    training_target_list.append(training_target_power_port_taped)

    ### Taped power port far
    glog.info(
        "Adding hacked/taped up far away view of the power port to the model list"
    )
    ideal_target_list.append(ideal_power_port_taped_far)
    training_target_list.append(training_target_power_port_taped_far)

    ### Taped power port night
    glog.info(
        "Adding hacked/taped up of the power port at night to the model list")
    ideal_target_list.append(ideal_power_port_taped_night)
    training_target_list.append(training_target_power_port_taped_night)

    ### Red Power Port
    #glog.info("Adding red power port to the model list")
    #ideal_target_list.append(ideal_power_port_red)
    #training_target_list.append(training_target_power_port_red)

    ### Red Loading Bay
    #glog.info("Adding red loading bay to the model list")
    #ideal_target_list.append(ideal_loading_bay_red)
    #training_target_list.append(training_target_loading_bay_red)

    ### Blue Power Port
    glog.info("Adding blue power port to the model list")
    ideal_target_list.append(ideal_power_port_blue)
    training_target_list.append(training_target_power_port_blue)

    ### Blue Loading Bay
    #glog.info("Adding blue loading bay to the model list")
    #ideal_target_list.append(ideal_loading_bay_blue)
    #training_target_list.append(training_target_loading_bay_blue)

    return ideal_target_list, training_target_list


def compute_target_definition():
    ideal_target_list, training_target_list = load_training_data()

    # Create feature extractor
    feature_extractor = tam.load_feature_extractor()

    # Use default parameters for now
    camera_params = camera_definition.load_camera_definitions()[0]

    for ideal_target in ideal_target_list:
        glog.debug("\nPreparing target for image %s" %
                   ideal_target.image_filename)
        ideal_target.extract_features(feature_extractor)
        ideal_target.filter_keypoints_by_polygons()
        ideal_target.compute_reprojection_maps()
        ideal_target.keypoint_list_3d = ideal_target.project_keypoint_to_3d_by_polygon(
            ideal_target.keypoint_list)

        if VISUALIZE_KEYPOINTS:
            for i in range(len(ideal_target.polygon_list)):
                ideal_pts_tmp = np.asarray(
                    ideal_target.polygon_list[i]).reshape(-1, 2)
                ideal_pts_3d_tmp = np.asarray(
                    ideal_target.polygon_list_3d[i]).reshape(-1, 3)
                # We can only compute pose if we have at least 4 points
                # Only matters for reprojection for visualization
                # Keeping this code here, since it's helpful when testing
                if (len(ideal_target.polygon_list[i]) >= 4):
                    img_copy = dtd.draw_polygon(ideal_target.image.copy(),
                                                ideal_target.polygon_list[i],
                                                (0, 255, 0), True)
                    dtd.visualize_reprojections(
                        img_copy, ideal_pts_tmp, ideal_pts_3d_tmp,
                        camera_params.camera_int.camera_matrix,
                        camera_params.camera_int.dist_coeffs)

            for polygon in ideal_target.polygon_list:
                img_copy = ideal_target.image.copy()
                kp_in_poly2d = []
                kp_in_poly3d = []
                for kp, kp_3d in zip(ideal_target.keypoint_list,
                                     ideal_target.keypoint_list_3d):
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
    ### TODO: Add code to do manual point selection
    ###############
    AUTO_PROJECTION = True

    if AUTO_PROJECTION:
        glog.debug(
            "Doing auto projection of training keypoints to 3D using ideal images"
        )
        # Match the captured training image against the "ideal" training image
        # and use those matches to pin down the 3D locations of the keypoints

        for target_ind in range(len(training_target_list)):
            # Assumes we have 1 ideal view for each training target
            training_target = training_target_list[target_ind]
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

            training_target.keypoint_list_3d = np.asarray(
                training_3d_list).reshape(-1, 1, 3)

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
    args = vars(ap.parse_args())

    VISUALIZE_KEYPOINTS = args["visualize"]
    if args["visualize"]:
        glog.info("Visualizing results")

    compute_target_definition()
