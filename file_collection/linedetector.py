import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.scan_width, self.scan_height = 200, 20
        self.image_width = 640
        self.area_width, self.area_height = 10, 5
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.roi_vertical_pos = 3

        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = self.roi_vertical_pos
        self.roi = self.cam_img[v:v + self.scan_height, :]

        # hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # avg_value = np.average(hsv[:, :, 2])
        # value_threshold = avg_value + 10
        # lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        # ubound = np.array([100, 255, 255], dtype=np.uint8)
        # self.mask = cv2.inRange(hsv, lbound, ubound)
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)

    def detect_lines(self):
        # Return positions of left and right lines detected.
        lmid, rmid = self.scan_width, self.image_width - self.scan_width

        pixel_cnt_threshold = [0.2 * self.area_width * self.area_height, 0.1 * self.area_height * self.area_width]

        bin_left, bin_right = -1, -1
        edge_left, edge_right = -1, -1

        for l in range(lmid - self.area_width, 0, -1):
            area = self.mask[self.row_begin:self.row_end, l:l + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold[0]:
                bin_left = l
                break

        for l in range(lmid - self.area_width, 0, -1):
            area = self.edge[self.row_begin:self.row_end, l:l + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold[1]:
                edge_left = l
                break

        for r in range(rmid, self.image_width - self.area_width):
            area = self.mask[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold[0]:
                bin_right = r
                break

        for r in range(rmid, self.image_width - self.area_width):
            area = self.edge[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold[1]:
                edge_right = r
                break

        left = -1
        right = -1

        if bin_left != -1 and edge_left != -1:
            left = int(0.5 * edge_left + 0.5 * bin_left)
        elif bin_left != -1 or edge_left != -1:
            left = max(bin_left, edge_left)

        if bin_right != -1 and edge_right != -1:
            right = int(0.5 * edge_right + 0.5 * bin_right)
        elif bin_right != -1 or edge_right != -1:
            right = max(bin_right, edge_right)

        return left, right

    def show_images(self, left, right):
        # Display images for debugging purpose;
        # do not forget to call cv2.waitKey().
        view = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)

        if left != -1:
            lsquare = cv2.rectangle(view,
                                    (left - self.area_width, self.row_begin),
                                    (left, self.row_end),
                                    (0, 255, 0), 3)
        else:
            print("Lost left line")

        if right != -1:
            rsquare = cv2.rectangle(view,
                                    (right, self.row_begin),
                                    (right + self.area_width, self.row_end),
                                    (0, 255, 0), 3)
        else:
            print("Lost right line")

        cv2.imshow("view", view)
        cv2.waitKey(1)

        if right == -1:
            right = self.lost_edgeline()

    def lost_edgeline(self):
        rmid = self.image_width - self.scan_width
        pixel_cnt_threshold = 5
        hsv = cv2.cvtColor(self., cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value + 30
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)
        bin_right = -1

        for r in range(rmid, self.image_width - self.area_width):
            area = self.mask[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                bin_right = r
                break

        return bin_right