import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import sys

class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.image_width = 640
        self.scan_width, self.scan_height = 200, 20
        self.rmid, self.lmid = self.scan_width, self.image_width - self.scan_width
        self.area_width, self.area_height = 20, 10
        self.roi_vertical_pos = 430
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height

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

        hsv = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, :])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(self.roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)

    def detect_lines(self):
        # Return positions of left and right lines detected.
        left, right = -1, -1
        pixel_cnt_threshold = 0.3 * self.area_width * self.area_height

        for i in range(self.area_width, self.lmid):
            hsvarea = self.mask[self.row_begin:self.row_end, i - self.area_width:i]
            cannyarea = self.edge[self.row_begin:self.row_end, i - self.area_width:i]
            if cv2.countNonZero(hsvarea) > pixel_cnt_threshold and cv2.countNonZero(cannyarea) > (pixel_cnt_threshold *0.5) :
                left = i
                break

        for i in range(self.image_width - self.area_width, self.rmid, -1):
            hsvarea = self.mask[self.row_begin:self.row_end, r:r+self.area_width]
            cannyarea = self.edge[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(hsvarea) > pixel_cnt_threshold and cv2.countNonZero(cannyarea) > (pixel_cnt_threshold*0.5):
                right = i
                break

        return left, right

    def show_images(self, left, right):
        # Display images for debugging purposes;

        if left != -1:
            lsquare = cv2.rectangle(self.cam_img, (left - self.area_width, self.row_begin), (left,self.row_end), (0,255,0),3)
        else:
            print("Lost left line")

        if right != -1:
            rsquare = cv2.rectangle(self.cam_img, (right, self.row_begin), (right + self.area_width, self.row_end), (0, 255, 0), 3)
        else:
            print("Lost right line")

        src = cv2.resize(self.cam_img, (640,360))
        dst = cv2.Canny(src, 50, 200, None, 3)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)
        lines = cv2.HoughLines(dst, 1, np.pi / 180, 150,None, 0,0)

        if lines is not None:
            for i in range(len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a*rho
                y0 = b*rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cdst,pt1,pt2,(0,0,255),3,cv2.LINE_AA)

        linesP = cv2.HoughLinesP(dst , 1, np.pi / 180, 50, None, 50, 10)

        if linesP is not None:
            for i in range(len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP,(l[0],l[1]),(l[2],l[3]),(0,0,255),3,cv2.LINE_AA)

        cv2.imshow("origin", self.cam_img)
        cv2.imshow("hough1", cdst)
        cv2.imshow("hough2", cdstP)

        # do not forget to call cv2.waitKey().
        if cv2.waitKey(1) & 0xFF == 27:
            cv2.destroyAllWindows()

