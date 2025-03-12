#!/usr/bin/env python3
"""
sift_template.py
Refactored object detection using SIFT with RANSAC and template matching.
"""

import cv2
import imutils
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0    X  > > > > >
#
#   Y
#
#   v   This is the image. Y increases downwards, X increases rightwards.
#   v   Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#   v   where (xmin, ymin) is the bottom left of the bbox and (xmax, ymax) is the top right.
###############################################################

def image_print(img, winname="Image"):
    """
    Display an image for debugging.
    Press any key to continue.
    """
    cv2.namedWindow(winname)
    cv2.moveWindow(winname, 40, 30)
    cv2.imshow(winname, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def ensure_gray(img):
    """
    Ensure an image is grayscale.
    """
    if len(img.shape) == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img

def compute_sift_features(img, sift):
    """
    Compute keypoints and descriptors using SIFT.
    """
    return sift.detectAndCompute(img, None)

def filter_matches_ratio(matches, ratio=0.75):
    """
    Filter matches using Lowe's ratio test.
    """
    good = []
    for m, n in matches:
        if m.distance < ratio * n.distance:
            good.append(m)
    return good

def get_transformed_bbox(template_shape, M):
    """
    Given template dimensions and homography M, compute the transformed bounding box.
    Returns bbox as ((xmin, ymax), (xmax, ymin)) to match the coordinate system:
      bottom left is (xmin, ymax) and top right is (xmax, ymin).
    """
    h, w = template_shape
    # Define template corners (clockwise order).
    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
    transformed_pts = cv2.perspectiveTransform(pts, M)
    
    xs = transformed_pts[:, 0, 0]
    ys = transformed_pts[:, 0, 1]
    x_min, x_max = np.min(xs), np.max(xs)
    y_min, y_max = np.min(ys), np.max(ys)
    # Remember: y increases downward, so bottom left is (x_min, y_max)
    return ((int(x_min), int(y_max)), (int(x_max), int(y_min)))

def cd_sift_ransac(img, template):
    """
    Detect object using SIFT and RANSAC.
    Input:
        img: np.array; image where to detect the object.
        template: np.array; the template image.
    Return:
        bbox: ((x1, y1), (x2, y2))
          where (x1, y1) is the bottom left and (x2, y2) is the top right.
    """
    MIN_MATCH = 10

    # Ensure images are grayscale.
    template_gray = ensure_gray(template)
    img_gray = ensure_gray(img)

    # Create a SIFT detector.
    sift = cv2.xfeatures2d.SIFT_create()

    # Compute keypoints and descriptors.
    kp_t, des_t = compute_sift_features(template_gray, sift)
    kp_i, des_i = compute_sift_features(img_gray, sift)

    # Debug: show keypoints.
    image_print(cv2.drawKeypoints(template_gray, kp_t, None, color=(0, 255, 0),
                                  flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS),
                winname="Template KeyPoints")
    image_print(cv2.drawKeypoints(img_gray, kp_i, None, color=(0, 255, 0),
                                  flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS),
                winname="Image KeyPoints")

    # Use BFMatcher with k=2 for knn match.
    matches = cv2.BFMatcher().knnMatch(des_t, des_i, k=2)
    good_matches = filter_matches_ratio(matches, ratio=0.75)

    if len(good_matches) > MIN_MATCH:
        # Debug: show good matches.
        match_img = cv2.drawMatches(template_gray, kp_t, img_gray, kp_i, good_matches, None,
                                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        image_print(match_img, "Good Matches")

        # Extract point coordinates from good matches.
        src_pts = np.float32([kp_t[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_i[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Compute homography using RANSAC.
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if M is None:
            print("[SIFT] Homography computation failed.")
            return ((0, 0), (0, 0))

        # Draw transformed bounding box on the original image for debugging.
        bbox = get_transformed_bbox(template_gray.shape, M)
        img_bbox = img.copy()
        pts = np.int32(cv2.perspectiveTransform(
                np.float32([[0, 0], [0, template_gray.shape[0]-1],
                            [template_gray.shape[1]-1, template_gray.shape[0]-1],
                            [template_gray.shape[1]-1, 0]]).reshape(-1, 1, 2), M))
        cv2.polylines(img_bbox, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
        image_print(img_bbox, "Detected Bounding Box")
        
        return bbox
    else:
        print("[SIFT] Not enough matches; found:", len(good_matches))
        return ((0, 0), (0, 0))

def cd_template_matching(img, template):
    """
    Detect object using a template matching algorithm.
    Input:
        img: np.array; image where to detect the object.
        template: np.array; the template image.
    Return:
        bbox: ((x1, y1), (x2, y2))
         where (x1, y1) is the bottom left and (x2, y2) is the top right.
    """
    template_edges = cv2.Canny(template, 50, 200)
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_edges = cv2.Canny(grey_img, 50, 200)
    imgH, imgW = img_edges.shape[:2]

    best_match = None  # (max_val, top_left, width, height)

    for scale in np.linspace(1.5, 0.5, 50):
        resized = imutils.resize(template_edges,
                                 width=int(template_edges.shape[1] * scale))
        h, w = resized.shape[:2]
        if h > imgH or w > imgW:
            continue

        result = cv2.matchTemplate(img_edges, resized, cv2.TM_CCOEFF_NORMED)
        (_, max_val, _, max_loc) = cv2.minMaxLoc(result)
        if best_match is None or max_val > best_match[0]:
            best_match = (max_val, max_loc, w, h)

    if best_match is not None:
        (_, top_left, w, h) = best_match
        # Translate coordinates:
        # top_left is upper left; bottom left is (x, top_left_y + h)
        bbox = ((top_left[0], top_left[1] + h), (top_left[0] + w, top_left[1]))
        return bbox
    else:
        print("[Template] No match found.")
        return ((0, 0), (0, 0))

if __name__ == '__main__':
    # Simple test examples (adjust file names as needed)
    test_img = cv2.imread("test_image.png")
    test_template = cv2.imread("test_template.png", cv2.IMREAD_GRAYSCALE)
    if test_img is None or test_template is None:
        print("Test images not found!")
    else:
        bbox_sift = cd_sift_ransac(test_img, test_template)
        bbox_template = cd_template_matching(test_img, test_template)
        print("SIFT bbox:", bbox_sift)
        print("Template matching bbox:", bbox_template)
        
        # Visualize SIFT bbox by drawing a rectangle.
        (pt_bl, pt_tr) = bbox_sift   # (bottom left, top right)
        cv2.rectangle(test_img, (pt_bl[0], pt_tr[1]), (pt_tr[0], pt_bl[1]),
                      (0, 255, 0), 2)
        image_print(test_img, "Final Result")
