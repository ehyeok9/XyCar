#!/usr/bin/env python

import cv2, time
import numpy as np


def steer(left, right):
    mid = (left + right) // 2

    if mid < 280:
        angle = int((320 - mid) * 0.25)
    elif mid > 360:
        angle = -int((mid - 320) * 0.25)
    else:
        angle = 0

    if left == -1 and right == -1:
        angle = -1
    elif left == -1:
        angle = -40
    elif right == -1:
        angle = 40

    return angle


def accelerate(angle):
    if angle == -1:
        speed = 5
    elif angle < -20 or angle > 20:
        speed = 10
    else:
        speed = 20
    return speed


cap = cv2.VideoCapture('2.avi')

value_threshold = 200

image_width = 640
scan_width, scan_height = 200, 20
lmid, rmid = scan_width, image_width - scan_width
area_width, area_height = 10, 5
roi_vertical_pos = 300
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_cnt_threshold = [0.2 * area_width * area_height, 0.1 * area_height * area_width]

while True:
    ret, frame = cap.read()
    if not ret:
        break
    if cv2.waitKey(1) & 0xFF == 27:
        break

    roi = frame[roi_vertical_pos:roi_vertical_pos + scan_height, :]
    frame = cv2.rectangle(frame, (0, roi_vertical_pos),
        (image_width - 1, roi_vertical_pos + scan_height),
        (255, 0, 0), 3)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edge = cv2.Canny(blur, 60, 70)

    value_threshold = np.average(hsv[:, :, 2]) + 30

    lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    bin = cv2.inRange(hsv, lbound, ubound)

    bin_view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)
    edge_view = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)

    # bin_left, bin_right = -1, -1
    edge_left, edge_right = -1, -1

    """
    for l in range(lmid - area_width, 0, -1):
        area = bin[row_begin:row_end, l:l + area_width]
        if cv2.countNonZero(area) > pixel_cnt_threshold[0]:
            bin_left = l
            break
    """

    for l in range(lmid - area_width, 0, -1):
        area = edge[row_begin:row_end, l:l + area_width]
        if cv2.countNonZero(area) > pixel_cnt_threshold[1]:
            edge_left =  l
            break
    """
    for r in range(rmid, image_width - area_width):
        area = bin[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > pixel_cnt_threshold[0]:
            bin_right = r
            break
    for r in range(rmid, image_width - area_width):
        area = edge[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > pixel_cnt_threshold[1]:
            edge_right = r
            break
    """
    left = edge_left
    # right = edge_right

    """
    if bin_left != -1 and edge_left != -1:
        left = int(0.5 * edge_left + 0.5 * bin_left)
    elif bin_left != -1 or edge_left != -1:
        left = max(bin_left, edge_left)

    if bin_right != -1 and edge_right != -1:
        right = int(0.5 * edge_right + 0.5 * bin_right)
    elif bin_right != -1 or edge_right != -1:
        right = max(bin_right, edge_right)
    """

    if left != -1:
        lsquare = cv2.rectangle(roi,
                                (left - area_width, row_begin),
                                (left, row_end),
                                (0, 255, 0), 3)
    else:
        # print("Lost left line", left, right)
        print("Lost left line", left)

    if right != -1:
        rsquare = cv2.rectangle(roi,
                                (right, row_begin),
                                (right + area_width, row_end),
                                (0, 255, 0), 3)
    else:
        print("Lost right line", left, right)

    cv2.imshow("origin", frame)
    cv2.imshow("edge", edge_view)
    # cv2.imshow("bin", bin_view)
    cv2.imshow("view", roi)
    # angle = steer(left, right)
    # print(90 + angle, accelerate(angle) + 90, "mid", (left + right) // 2, "left", left, "right", right)
    cv2.waitKey(50)

cap.release()
cv2.destroyAllWindows()

"""
    for l in range(area_width, lmid):
        area = bin[row_begin:row_end, l - area_width:l]
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            left = l
            break

    for r in range(image_width - area_width, rmid, -1):
        area = bin[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            right = r
            break
"""