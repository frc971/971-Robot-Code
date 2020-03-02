import cv2
import glog
import math
import numpy as np
import time

### DEFINITIONS
MIN_MATCH_COUNT = 10  # 10 is min; more gives better matches
FEATURE_EXTRACTOR_NAME = 'SIFT'
QUERY_INDEX = 0  # We use a list for both training and query info, but only ever have one query item

glog.setLevel("WARN")


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
# Input: R: numpy 3x3 rotation matrix
# Output: numpy 3x1 vector of Euler angles (x,y,z)
def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# Load images based on list of image names
# Return list containing loaded images
def load_images(image_names):
    image_list = []
    for im in image_names:
        # Load image (in color; let opencv convert to B&W for features)
        img_data = cv2.imread(im)
        if img_data is None:
            glog.error("Failed to load image: ", im)
            exit()
        else:
            image_list.append(img_data)

    return image_list


# Load feature extractor based on extractor name
# Returns feature extractor object
def load_feature_extractor():
    if FEATURE_EXTRACTOR_NAME is 'SIFT':
        # Initiate SIFT detector
        feature_extractor = cv2.xfeatures2d.SIFT_create()
    elif FEATURE_EXTRACTOR_NAME is 'SURF':
        # Initiate SURF detector
        feature_extractor = cv2.xfeatures2d.SURF_create()
    elif FEATURE_EXTRACTOR_NAME is 'ORB':
        # Initiate ORB detector
        feature_extractor = cv2.ORB_create()

    return feature_extractor


# Run feature detector and compute feature descriptions on image_list
# Input: feature_extractor object
#        image_list: list containing images
# Return: Lists of keypoints and lists of feature descriptors, one for each image
def detect_and_compute(feature_extractor, image_list):
    descriptor_lists = []
    keypoint_lists = []
    if (FEATURE_EXTRACTOR_NAME in ('SIFT', 'SURF')):
        for i in range(len(image_list)):
            # find the keypoints and descriptors with SIFT
            kp, des = feature_extractor.detectAndCompute(image_list[i], None)
            descriptor_lists.append(des)
            keypoint_lists.append(kp)
    elif FEATURE_EXTRACTOR_NAME is 'ORB':
        # TODO: Check whether ORB extractor can do detectAndCompute.
        # If so, we don't need to have this branch for ORB
        for i in range(len(image_list)):
            # find the keypoints and descriptors with ORB
            kp = feature_extractor.detect(image_list[i], None)
            keypoint_lists.append(kp)
            des = feature_extractor.compute(image_list[i], None)
            descriptor_lists.append(des)

    return keypoint_lists, descriptor_lists


# Given a keypoint descriptor list, create a matcher
# Input: descriptor_lists: List of descriptors, one for each training image
# Returns: a keypoint matcher trained on all descriptors, indexed by image
def train_matcher(descriptor_lists):
    if (FEATURE_EXTRACTOR_NAME in ('SIFT', 'SURF')):
        # Use FLANN KD tree for SIFT & SURF
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)

        matcher = cv2.FlannBasedMatcher(index_params, search_params)
        matcher.add(descriptor_lists)
        matcher.train()
    elif FEATURE_EXTRACTOR_NAME is 'ORB':
        # Use FLANN LSH for ORB
        FLANN_INDEX_LSH = 6
        index_params = dict(
            algorithm=FLANN_INDEX_LSH,
            table_number=6,  # 12
            key_size=12,  # 20
            multi_probe_level=2)  #2
        search_params = dict(checks=50)
        matcher = cv2.FlannBasedMatcher(index_params, search_params)
        # Other option: BFMatcher with default params
        # NOTE: I think Brute Force matcher can only do 1 match at a time,
        # rather than loading keypoints from all model images
        #matcher = cv2.BFMatcher()

    return matcher


# Given a matcher and the query descriptors (and for ORB the training list),
# Compute the best matches, filtering using the David Lowe magic ratio of 0.7
# Return list of good match lists (one set of good matches for each training image for SIFT/SURF)
def compute_matches(matcher, train_descriptor_lists, query_descriptor_lists):

    # We'll create a match list for each of the training images
    good_matches_list = []
    if (FEATURE_EXTRACTOR_NAME in ('SIFT', 'SURF')):
        # We're just doing one query at a time, so grab the first from the list
        desc_query = query_descriptor_lists[QUERY_INDEX]
        matches = matcher.knnMatch(desc_query, k=2)

        for train_index in range(len(train_descriptor_lists)):
            # store all the good matches as per Lowe's ratio test.
            # (Lowe originally proposed 0.7 ratio, but 0.75 was later proposed as a better option.  We'll go with the more conservative (fewer, better matches) for now)
            good_matches = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance and m.imgIdx is train_index:
                    good_matches.append(m)

            good_matches_list.append(good_matches)

    elif FEATURE_EXTRACTOR_NAME is 'ORB':
        matches = matcher.knnMatch(train_keypoint_lists[0], desc_query, k=2)
        good_matches = []
        for m in matches:
            if m:
                if len(m) == 2:
                    if m[0].distance < 0.7 * m[1].distance:
                        good_matches.append(m[0])

        good_matches_list.append(good_matches)

    return good_matches_list


# Given a set of training keypoints lists, and a query keypoint list,
# and the good matches for each training<->query match, returns the
# homography for each pair and a mask list of matchces considered good
# by the homography
# Inputs: train_keypoint_lists: List of keypoints lists from training images
#         query_keypoint_lists: Keypoint list (only 1) from query image
#         good_matches_list: List of matches for each training image -> query
# Returns: homography_list: List of each homography between train->query
#                           Returns [] if not enough matches / bad homography
#          matches_mask_list: Mask list of matches kept by homography for each
#                             set of matches
def compute_homographies(train_keypoint_lists, query_keypoint_lists,
                         good_matches_list):
    homography_list = []
    matches_mask_list = []
    for i in range(len(train_keypoint_lists)):
        good_matches = good_matches_list[i]
        if len(good_matches) < MIN_MATCH_COUNT:
            glog.warn(
                "Not enough matches are for model %d: %d out of needed #: %d" %
                (i, len(good_matches), MIN_MATCH_COUNT))
            homography_list.append([])
            matches_mask_list.append([])
            continue

        glog.info(
            "Got good number of matches for model %d: %d (needed only %d)" %
            (i, len(good_matches), MIN_MATCH_COUNT))
        # Extract and bundle keypoint locations for computations
        src_pts = np.float32([
            train_keypoint_lists[i][m.trainIdx].pt for m in good_matches
        ]).reshape(-1, 1, 2)
        dst_pts = np.float32([
            query_keypoint_lists[QUERY_INDEX][m.queryIdx].pt
            for m in good_matches
        ]).reshape(-1, 1, 2)
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
        matches_mask = mask.ravel().tolist()
        homography_list.append(H)
        matches_mask_list.append(matches_mask)

    return homography_list, matches_mask_list


# Helper function to display images
# Shows side-by-side panel with query on left, training on right
# connects keypoints between two and draws target on query
# Also shows image with query unwarped (to match training image) and target pt
def show_results(training_images, train_keypoint_lists, query_images,
                 query_keypoint_lists, target_point_list, good_matches_list):
    glog.info("Showing results for ", len(training_images), " training images")

    homography_list, matches_mask_list = compute_homographies(
        train_keypoint_lists, query_keypoint_lists, good_matches_list)
    for i in range(len(train_keypoint_lists)):
        good_matches = good_matches_list[i]
        if len(good_matches) < MIN_MATCH_COUNT:
            continue
        glog.debug("Showing results for model ", i)
        matches_mask_count = matches_mask_list[i].count(1)
        if matches_mask_count != len(good_matches):
            glog.info("Homography rejected some matches!  From ",
                      len(good_matches), ", only ", matches_mask_count,
                      " were used")

        if matches_mask_count < MIN_MATCH_COUNT:
            glog.info(
                "Skipping match because homography rejected matches down to below ",
                MIN_MATCH_COUNT)
            continue

        # Create a box based on the training image to map onto the query img
        h, w, ch = training_images[i].shape
        H = homography_list[i]
        train_box_pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                                    [w - 1, 0]]).reshape(-1, 1, 2)
        query_box_pts = cv2.perspectiveTransform(train_box_pts, H)

        # Figure out where the training target goes on the query img
        transformed_target = cv2.perspectiveTransform(
            target_point_list[i].reshape(-1, 1, 2), H)
        # Ballpark the size of the circle so it looks right on image
        radius = int(
            32 * abs(H[0][0] + H[1][1]) / 2)  # Average of scale factors
        # We're only using one query image at this point
        query_image = query_images[QUERY_INDEX].copy()

        # Draw training box and target points on the query image
        query_image = cv2.polylines(query_image, [np.int32(query_box_pts)],
                                    True, (0, 255, 0), 3, cv2.LINE_AA)
        query_image = cv2.circle(
            query_image,
            (transformed_target.flatten()[0], transformed_target.flatten()[1]),
            radius, (0, 255, 0), 3)

        # Draw the matches and show it
        draw_params = dict(
            matchColor=(0, 255, 0),  # draw matches in green color
            singlePointColor=None,
            matchesMask=matches_mask_list[i],  # draw only inliers
            flags=2)

        img3 = cv2.drawMatches(query_image, query_keypoint_lists[QUERY_INDEX],
                               training_images[i], train_keypoint_lists[i],
                               good_matches_list[i], None, **draw_params)
        glog.debug("Drawing matches for model ", i,
                   ".  Query on left, Training image on right")
        cv2.imshow('Matches', img3), cv2.waitKey()

        # Next, unwarp the query image so it looks like the training view
        H_inv = np.linalg.inv(H)
        query_image_warp = cv2.warpPerspective(query_image, H_inv, (w, h))
        cv2.imshow('Unwarped Image', query_image_warp), cv2.waitKey()

    # Go ahead and return these, for use elsewhere
    return homography_list, matches_mask_list


# Helper function to display keypoints
# Input: image
#        keypoint_list: List of opencv keypoints
def show_keypoints(image, keypoint_list):
    ret_img = image.copy()
    ret_img = cv2.drawKeypoints(
        ret_img,
        keypoint_list,
        ret_img,
        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("Keypoints", ret_img)
    cv2.waitKey(0)
    return ret_img
