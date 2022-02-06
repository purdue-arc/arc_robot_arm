#!/usr/bin/env python3
import numpy as np

import cv2


def _get_color_mask(frame, hsv_color_range):
    """Convert frame to HSV and and threshold image to given color range in HSV.

    Arguments:
    hsv_color_range -- tuple of np arrays (lower color bound, upper color bound)
    frame -- image to generate mask in np array format
    """
    lower_bound, upper_bound = hsv_color_range

    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsvFrame, lower_bound, upper_bound)

    # Morphological Transform, Dilation
    kernal = np.ones((5, 5), "uint8")
    mask = cv2.dilate(mask, kernal)

    return mask


def get_bbox_color(frame, hsv_color_range):
    """Create contour around color mask of given range and return rectangle bounding box.

    Arguments:
    hsv_color_range -- tuple of np arrays (lower color bound, upper color bound)
    frame -- image to generate mask in np array format
    """
    if frame is not None:
        mask = _get_color_mask(frame, hsv_color_range)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) != 0:
            contour = contours[0]
            if cv2.contourArea(contour):
                bbox = cv2.boundingRect(contour)
                return bbox