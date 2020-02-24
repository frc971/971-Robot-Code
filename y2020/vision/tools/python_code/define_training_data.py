import argparse
import cv2
import glog
import json
import math
import numpy as np
import time

import train_and_match as tam

# Points for current polygon
point_list = []
current_mouse = (0, 0)


def get_mouse_event(event, x, y, flags, param):
    global point_list
    global current_mouse
    current_mouse = (x, y)
    if event == cv2.EVENT_LBUTTONUP:
        glog.debug("Adding point at %d, %d" % (x,y))
        point_list.append([x, y])
    pass


def draw_polygon(image, polygon, color=(255, 0, 0), close_polygon=False):
    for point in polygon:
        image = cv2.circle(image, (point[0], point[1]), 5, (255, 0, 0), -1)
    if (len(polygon) > 1):
        np_poly = np.array(polygon)
        image = cv2.polylines(
            image, [np_poly], close_polygon, color, thickness=3)
    return image


# Close out polygon, return True if size is 3 or more points
def finish_polygon(image, polygon):
    global point_list
    # If we have at least 3 points, close and show the polygon
    if len(point_list) <= 2:
        return False

    point_list.append(point_list[0])
    image = draw_polygon(image, point_list, color=(0, 0, 255))
    cv2.imshow("image", image)
    cv2.waitKey(500)
    return True


def define_polygon(image):
    global point_list
    # Set of all defined polygons
    point_list = []
    polygon_list = []

    display_image = image.copy()

    cv2.namedWindow("image")
    cv2.setMouseCallback("image", get_mouse_event)

    while True:
        cv2.imshow("image", display_image)
        key = cv2.waitKey(1) & 0xFF

        # if the 'r' key is pressed, reset the current polygon
        # Leaves previously defined polygons intact
        if key == ord("r"):
            display_image = image.copy()
            point_list = []

        # if the 'd' key is pressed, delete the last point
        if key == ord("d"):
            display_image = image.copy()
            point_list.pop()

        # if the 'n' key is pressed, save current polygon, and move to next
        if key == ord("n"):
            if (finish_polygon(display_image, point_list)):
                polygon_list.append(point_list)
            display_image = image.copy()
            point_list = []

        # if the 'q' key is pressed, break from the loop and finish polygon
        elif key == ord("q"):
            if (finish_polygon(display_image, point_list)):
                polygon_list.append(point_list)
            break

        display_image = draw_polygon(display_image, point_list)

    return polygon_list


# Given a list of points on an image, prompt the user to click on the
# corresponding points within the image.
# Return the list of those points that have been clicked
def define_points_by_list(image, points):
    global point_list
    global current_mouse
    point_list = []

    display_image = image.copy()

    cv2.namedWindow("image")
    cv2.setMouseCallback("image", get_mouse_event)

    while (len(point_list) < len(points)):
        i = len(point_list)
        # Draw mouse location and suggested target
        display_image = image.copy()
        display_image = cv2.circle(display_image, (points[i][0], points[i][1]),
                                   15, (0, 255, 0), 2)
        cursor_length = 5
        display_image = cv2.line(
            display_image,
            (current_mouse[0] - cursor_length, current_mouse[1]),
            (current_mouse[0] + cursor_length, current_mouse[1]), (255, 0, 0),
            2, cv2.LINE_AA)
        display_image = cv2.line(
            display_image,
            (current_mouse[0], current_mouse[1] - cursor_length),
            (current_mouse[0], current_mouse[1] + cursor_length), (255, 0, 0),
            2, cv2.LINE_AA)

        cv2.imshow("image", display_image)

        key = cv2.waitKey(1) & 0xFF

        # if the 'r' key is pressed, reset the current point collection
        # Leaves previously defined polygons intact
        if key == ord("r"):
            draw_image = image.copy()
            point_list = []

        # if the 'd' key is pressed, delete the last point
        elif key == ord("d"):
            draw_image = image.copy()
            point_list.pop()

        # if the 'q' key is pressed, quit
        elif key == ord("q"):
            quit()

    return point_list

# Determine whether a given point lies within (or on border of) a set of polygons
# Return true if it does
def point_in_polygons(point, polygons):
    for poly in polygons:
        np_poly = np.asarray(poly)
        dist = cv2.pointPolygonTest(np_poly, (point[0], point[1]), True)
        if dist >=0:
            return True

    return False

## Filter keypoints by polygons
def filter_keypoints_by_polygons(keypoint_list, descriptor_list, polygons):
    # TODO: Need to make sure we've got the right numpy array / list 
    keep_keypoint_list = []
    keep_descriptor_list = []
    reject_keypoint_list = []
    reject_descriptor_list = []
    keep_list = []
    reject_list = []

    # For now, pretend keypoints are just points
    for i in range(len(keypoint_list)):
        if point_in_polygons((keypoint_list[i].pt[0], keypoint_list[i].pt[1]), polygons):
            keep_list.append(i)
        else:
            reject_list.append(i)

    keep_keypoint_list = [keypoint_list[kp_ind] for kp_ind in keep_list]
    reject_keypoint_list = [keypoint_list[kp_ind] for kp_ind in reject_list]
    # Allow function to be called with no descriptors, and just return empty list
    if descriptor_list is not None:
        keep_descriptor_list = descriptor_list[keep_list,:]
        reject_descriptor_list = descriptor_list[reject_list,:]

    return keep_keypoint_list, keep_descriptor_list, reject_keypoint_list, reject_descriptor_list, keep_list

# Helper function that appends a column of ones to a list (of 2d points)
def append_ones(point_list):
    return np.hstack([point_list, np.ones((len(point_list),1))])

# Given a list of 2d points and thei corresponding 3d locations, compute map
# between them.
# pt_3d = (pt_2d, 1) * reprojection_map
# TODO: We should really look at residuals to see if things are messed up
def compute_reprojection_map(polygon_2d, polygon_3d):
    pts_2d = np.asarray(np.float32(polygon_2d)).reshape(-1,2)
    pts_2d_lstsq = append_ones(pts_2d)
    pts_3d_lstsq = np.asarray(np.float32(polygon_3d)).reshape(-1,3)

    reprojection_map = np.linalg.lstsq(pts_2d_lstsq, pts_3d_lstsq, rcond=-1)[0]

    return reprojection_map

# Given set of keypoints (w/ 2d location), a reprojection map, and a polygon
# that defines regions for where this is valid
# Returns a numpy array of 3d locations the same size as the keypoint list
def compute_3d_points(keypoint_2d_list, reprojection_map):
    pt_2d_lstsq = append_ones(np.asarray(np.float32(keypoint_2d_list)).reshape(-1,2))
    pt_3d = pt_2d_lstsq.dot(reprojection_map)

    return pt_3d

# Given 2d and 3d locations, and camera location and projection model,
# display locations on an image
def visualize_reprojections(img, pts_2d, pts_3d, cam_mat, distortion_coeffs):
    # Compute camera location
    # TODO: Warn on bad inliers
    # TODO: Change this to not have to recast to np
    pts_2d_np = np.asarray(np.float32(pts_2d)).reshape(-1, 1, 2)
    pts_3d_np = np.asarray(np.float32(pts_3d)).reshape(-1, 1, 3)
    retval, R, T, inliers = cv2.solvePnPRansac(pts_3d_np, pts_2d_np, cam_mat,
                                               distortion_coeffs)
    pts_3d_proj_2d, jac_2d = cv2.projectPoints(pts_3d_np, R, T, cam_mat,
                                               distortion_coeffs)
    if inliers is None:
        glog.warn("WARNING: Didn't get any inliers when reprojecting polygons")
        return img
    for i in range(len(pts_2d)):
        pt_2d = pts_2d_np[i][0]
        pt_3d_proj = pts_3d_proj_2d[i][0]
        pt_color = (0, 255, 0)
        if i not in inliers:
            pt_color = (0, 0, 255)

        img = cv2.circle(img, (pt_2d[0], pt_2d[1]), 3, pt_color, 3)
        img = cv2.circle(img, (pt_3d_proj[0], pt_3d_proj[1]), 15, pt_color, 3)

    cv2.imshow("image", img)
    cv2.waitKey(0)
    return img


def sample_define_polygon_usage():
    image = cv2.imread("test_images/train_power_port_red.png")

    polygon_list = define_polygon(image)
    glog.debug(polygon_list)


def sample_define_points_by_list_usage():
    image = cv2.imread("test_images/train_power_port_red.png")

    test_points = [(451, 679), (451, 304),
                   (100, 302), (451, 74),
                   (689, 74), (689, 302),
                   (689, 679)]

    polygon_list = define_points_by_list(image, test_points)
    glog.debug(polygon_list)

