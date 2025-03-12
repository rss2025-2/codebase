#!/usr/bin/env python
import cv2
import numpy as np

def image_print(img):
    """
    Helper function to print out images, for debugging. Pass them in as a list.
    Press any key to continue.
    """
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template: (Not used here)
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
              (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([20, 255, 255])
    
    mask = cv2.inRange(hsv_img, lower_orange, upper_orange)
    
    image_print(mask)
    
    kernel = np.ones((7,7), np.uint8)
    mask_clean = cv2.erode(mask, kernel, iterations=2)
    mask_clean = cv2.dilate(mask_clean, kernel, iterations=4)
    
    image_print(mask_clean)
    
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        image_print(img)
        return ((0, 0), (0, 0))
    
    largest_contour = max(contours, key=cv2.contourArea)
    
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    bounding_box = ((x, y), (x + w, y + h))
    
    debug_img = img.copy()
    cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)
    image_print(debug_img)
        
    return bounding_box
