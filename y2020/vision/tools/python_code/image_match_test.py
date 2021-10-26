import cv2
import math
import numpy as np
import time

import field_display
import train_and_match as tam
import target_definition
import camera_definition

### DEFINITIONS
camera_params = camera_definition.load_pi1_camera_params()
target_list = target_definition.compute_target_definition(camera_params)

# Put list of all possible images we want to test, and then we'll select one
# Note, if you query on the same image as training, only that image will match
query_image_names = [
    'test_images/train_power_port_red.png',  #0
    'test_images/train_power_port_blue.png',  #1
    'test_images/test_game_manual.png',  #2
    'test_images/test_train_shift_left_100.png',  #3
    'test_images/test_train_shift_right_100.png',  #4
    'test_images/test_train_down_2x.png',  #5
    'test_images/test_train_down_4x.png',  #6
    'test_images/test_raspi3_sample.jpg',  #7
    'test_images/test_VR_sample1.png',  #8
    'test_images/train_loading_bay_blue.png',  #9
    'test_images/train_loading_bay_red.png',  #10
    'test_images/pi-7971-3_test_image.png',  #11
    'test_images/test_taped_dusk-2021-04-03-19-30-00.png',  #12
]

training_image_index = 0
# TODO: Should add argParser here to select this
query_image_index = 12  # Use -1 to use camera capture; otherwise index above list

##### Let's get to work!

training_image_names = []
training_images = []
train_keypoint_lists = []
train_descriptor_lists = []
target_points_tmp = []
target_pt_list = None
# Popluate the data structures used by this function
for target in target_list:
    # Image names
    print("Target filename:", target.image_filename)
    training_image_names.append(target.image_filename)
    # The training images
    training_images.append(target.image)
    # Keypoints and desciptors
    train_keypoint_lists.append(target.keypoint_list)
    train_descriptor_lists.append(target.descriptor_list)
    target_points_tmp.append(target.target_point_2d)

target_pt_list = np.asarray(target_points_tmp).reshape(-1, 1, 2)

# # Create the matcher.  This only needs to be created once
ts = time.monotonic()  # Do some basic timing
matcher = tam.train_matcher(train_descriptor_lists)
tf = time.monotonic()
print("Done training matcher, took ", tf - ts, " secs")
for i in range(len(train_keypoint_lists)):
    print("Model ", i, " has ", len(train_keypoint_lists[i]), " features: ")

# Create feature extractor
# TODO<Jim>: Should probably make sure we're using the same extractor for
# training and query
feature_extractor = tam.load_feature_extractor()

# Load the query image in.  Based on index in our list, or using camera if index is -1
query_images = []

if (query_image_index is -1):
    # Based on /dev/videoX setting
    CAMERA_INDEX = 2
    print("#### Using camera at /dev/video%d" % CAMERA_INDEX)
    # Open the device at the ID X for /dev/videoX
    cap = cv2.VideoCapture(CAMERA_INDEX)

    # Check whether user selected camera is opened successfully.
    if not (cap.isOpened()):
        print("Could not open video device")
        quit()

    # Capture frame-by-frame
    ret, frame = cap.read()
    query_images.append(frame)

else:
    # Load default images
    query_images = tam.load_images([query_image_names[query_image_index]])

# For storing out pose to measure noise
log_file = open("pose.out", 'w')

looping = True
# Grab images until 'q' is pressed (or just once for canned images)
while looping:
    if (query_image_index is -1):
        # Capture frame-by-frame
        ret, frame = cap.read()
        query_images[0] = frame
    else:
        looping = False

    # Extract features from query image
    query_keypoint_lists, query_descriptor_lists = tam.detect_and_compute(
        feature_extractor, query_images)
    print("Query image has ", len(query_keypoint_lists[0]), " features")

    ts = time.monotonic()
    good_matches_list = tam.compute_matches(matcher, train_descriptor_lists,
                                            query_descriptor_lists)
    tf = time.monotonic()
    print("Done finding matches (took ", tf - ts, " secs)")
    for i in range(len(train_keypoint_lists)):
        print("Model ", i, " has # good matches: ", len(good_matches_list[i]))

    # If we're querying static images, show full results
    if (query_image_index is not -1):
        print("Showing results for static query image")
        cv2.imshow('test', query_images[0]), cv2.waitKey(0)
        homography_list, matches_mask_list = tam.show_results(
            training_images, train_keypoint_lists, query_images,
            query_keypoint_lists, target_pt_list, good_matches_list)

    # Next, find homography (map between training and query images)
    homography_list, matches_mask_list = tam.compute_homographies(
        train_keypoint_lists, query_keypoint_lists, good_matches_list)

    # Image used to store field plot
    img_ret = None
    for i, good_matches in enumerate(good_matches_list):
        # TODO: Should look for best match after homography and display that one
        # OR: show results on all good matches
        print("Processing match for model %d" % i)

        if matches_mask_list[i].count(1) < tam.MIN_MATCH_COUNT:
            print(
                "Not continuing pose calc because not enough good matches after homography-- only had ",
                matches_mask_list[i].count(1))
            continue

        # Next, let's go to compute pose
        # I've chosen some coordinate frames to help track things.  Using indices:
        #   w:    world coordinate frame (at center wall of driver station)
        #   tarj: camera pose when jth target (e.g., power panel) was captured
        #   ci:   ith camera
        #   b:    body frame of robot (robot location)
        #
        # And, T for translation, R for rotation, "est" for estimated
        # So, T_w_b_est is the estimated translation from world to body coords
        #
        # A few notes:
        #   -- CV uses rotated frame, where z points out of camera, y points down
        #   -- This causes a lot of mapping around of points, but overall OK

        src_pts_3d = []
        query_image_matches = query_images[0].copy()
        for m in good_matches:
            src_pts_3d.append(target_list[i].keypoint_list_3d[m.trainIdx])
            pt = query_keypoint_lists[0][m.queryIdx].pt
            query_image_matches = cv2.circle(query_image_matches,
                                             (int(pt[0]), int(pt[1])), 5,
                                             (0, 255, 0), 3)

        cv2.imshow('DEBUG matches', query_image_matches), cv2.waitKey(0)
        # Reshape 3d point list to work with computations to follow
        src_pts_3d_array = np.asarray(np.float32(src_pts_3d)).reshape(-1, 3, 1)

        # Load from camera parameters
        cam_mat = camera_params.camera_int.camera_matrix
        cam_mat[0][0] = 390
        cam_mat[1][1] = 390
        # TODO<Jim>: Would be good to read this from a config file
        print("FIXING CAM MAT to pi camera: ", cam_mat)
        dist_coeffs = camera_params.camera_int.dist_coeffs

        # Create list of matching query point locations
        dst_pts = np.float32([
            query_keypoint_lists[0][m.queryIdx].pt for m in good_matches
        ]).reshape(-1, 1, 2)

        ts = time.monotonic()
        # TODO: May want to supply it with estimated guess as starting point
        # Find offset from camera to original location of camera relative to target
        retval, R_ci_w_estj, T_ci_w_estj, inliers = cv2.solvePnPRansac(
            src_pts_3d_array, dst_pts, cam_mat, dist_coeffs)

        tf = time.monotonic()
        print("Solving Pose took ", tf - ts, " secs")
        print("Found ", len(inliers), " inliers, out of ", len(dst_pts),
              " matched points")
        ### THIS is the estimate of the robot on the field!
        R_w_ci_estj, T_w_ci_estj = field_display.invert_pose_Rodrigues(
            R_ci_w_estj, T_ci_w_estj)
        #print("Pose from PnP is:\n", R_ci_w_estj, "\n", T_ci_w_estj)
        # The rotation estimate is for camera with z pointing forwards.
        # Compute the rotation as if x is forward
        R_w_ci_estj_robot = field_display.camera_rot_to_world_Rodrigues(
            R_w_ci_estj)
        #print("Pose in world coords is:\n", R_w_ci_estj, "\n", T_w_ci_estj,
        #      "\nWith robot coord frame rotation as \n", R_w_ci_estj_robot)

        # Use the rotational component about the z axis to define the heading
        # TODO<jim>: Change this to use rotation of x-unit vector
        heading_est = R_w_ci_estj_robot[2][0]
        # Plot location of the robot, along with heading
        color = (255, 0, 0)  # Blue
        if i in (0, 1):
            color = (0, 0, 255)  # Red
        img_ret = field_display.plot_bot_on_field(img_ret, color, T_w_ci_estj,
                                                  heading_est)

        # Compute vector to target
        T_w_target_pt = np.array(target_list[i].target_position).reshape(3, 1)
        vector_to_target = T_w_target_pt - T_w_ci_estj
        # Compute distance to target (full 3D)
        distance_to_target = np.linalg.norm(vector_to_target)
        # Compute distance to target (x,y)
        distance_to_target_ground = np.linalg.norm(vector_to_target[0:2])
        #print("Distance comparison: all: ", distance_to_target, " vs. ground: ", distance_to_target_ground)

        angle_to_target_abs = math.atan2(vector_to_target[1][0],
                                         vector_to_target[0][0])
        angle_to_target_robot = angle_to_target_abs - heading_est
        img_ret = field_display.plot_line_on_field(img_ret, (255, 255, 0),
                                                   T_w_ci_estj, T_w_target_pt)
        # THESE ARE OUR ESTIMATES OF HEADING, DISTANCE, AND SKEW TO TARGET
        log_file.write(
            '%lf, %lf, %lf\n' %
            (heading_est, distance_to_target_ground, angle_to_target_robot))
        print("Estimates: \n  Heading on field: ",
              "{:.2f}".format(heading_est), " radians; ",
              "{:.2f}".format(heading_est / math.pi * 180.0),
              " degrees\n  Distance to target: ",
              "{:.2f}".format(distance_to_target_ground),
              "m\n  Angle to target: ", "{:.2f}".format(angle_to_target_robot),
              " radians; ", "{:.2f}".format(angle_to_target_robot / math.pi *
                                            180.0), " degrees\n")

        # A bunch of code to visualize things...
        #
        # Draw target on the image
        h, w, ch = query_images[0].shape
        query_image_copy = query_images[0].copy()
        # Map target_pt onto image, for display
        target_point_2d_trans = cv2.perspectiveTransform(
            target_pt_list[i].reshape(-1, 1, 2),
            homography_list[i]).astype(int)
        # Ballpark the size of the circle so it looks right on image
        radius = int(12 *
                     abs(homography_list[i][0][0] + homography_list[i][1][1]) /
                     2)  # Average of scale factors
        query_image_copy = cv2.circle(query_image_copy,
                                      (target_point_2d_trans.flatten()[0],
                                       target_point_2d_trans.flatten()[1]),
                                      radius, (0, 255, 0), 3)

        # Create empty image the size of the field
        img_heading = field_display.load_field_image()
        img_heading[:, :, :] = 0
        f_h, f_w, f_ch = img_heading.shape

        # Create heading view, and paste it to bottom of field
        img_heading = field_display.plot_camera_to_target(
            img_heading, (0, 255, 0), heading_est, distance_to_target_ground,
            angle_to_target_robot)
        vis = np.concatenate((img_ret, img_heading), axis=0)

        # Paste query image to right of other views (scale to keep aspect ratio)
        img_query_scaled = cv2.resize(query_image_copy,
                                      (int(2 * w * f_h / h), 2 * f_h))
        vis = np.concatenate((vis, img_query_scaled), axis=1)

        # Scale down to make it fit on screen
        vis = cv2.resize(vis, (int(vis.shape[1] / 4), int(vis.shape[0] / 4)))
        cv2.imshow('field_display', vis)

        #Waits for a user input to quit the application
        pause_time = 0
        if (query_image_index is -1):
            pause_time = 1
        if cv2.waitKey(pause_time) & 0xFF == ord('q'):
            break

# Done.  Clean things up
if (query_image_index is -1):
    # When everything done, release the capture
    cap.release()

cv2.destroyAllWindows()
