import cv2
import numpy as np


def create_mareker_with_white_borders(marker):
    w, h, _ = marker.shape
    range = 50
    white = np.ndarray((w+2*range, h+2*range, 3), np.uint8)
    white.fill(255)
    white[range:range+h, range:range+w] = marker
    return white

#walls
for x in range(1, 10):
    for y in range(2):
        img_name = "walls/" + str(x) + str(y) + ".png"
        marker = cv2.imread("original_" + img_name)
        marker_with_borders = create_mareker_with_white_borders(marker)
        cv2.imwrite(img_name, marker_with_borders)

# platforms
for x in range(6):
    img_name = "platforms/" + str(x) + ".png"
    marker = cv2.imread("original_" + img_name)
    marker_with_borders = create_mareker_with_white_borders(marker)
    cv2.imwrite(img_name, marker_with_borders)

#goals
for x in range(244, 250):
    img_name = "goals/" + str(x) + ".png"
    marker = cv2.imread("original_" + img_name)
    marker_with_borders = create_mareker_with_white_borders(marker)
    cv2.imwrite(img_name, marker_with_borders)
