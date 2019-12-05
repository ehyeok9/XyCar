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
        self.area_width, self.area_height = 20, 20
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.roi_vertical_pos = 340

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

	"""
	hsv = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])        	
        value_threshold = 235
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)
	"""
        gray = cv2.cvtColor(self.roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)
	

    def detect_lines(self):
        # Return positions of left and right lines detected.
        lmid, rmid = self.scan_width, self.image_width - self.scan_width

        pixel_cnt_threshold = 21
        
        left = -1
        for l in range(lmid - self.area_width, 0, -1):
            area = self.edge[self.row_begin:self.row_end, l:l + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                left =  l
                break


        
        right = -1
        for r in range(440, self.image_width - self.area_width):
            area = self.edge[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > 10:
                right = r
                break
	print("left : " ,str(left)) 
	return left, right

    def show_images(self, left,right):
        # Display images for debugging purpose;
        # do not forget to call cv2.waitKey().
        view =  cv2.cvtColor(self.edge, cv2.COLOR_GRAY2BGR)
	#viewa = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
        if left != -1:
            lsquare = cv2.rectangle(view,
                                    (left - self.area_width, self.row_begin),
                                    (left, self.row_end),
                                    (0, 255, 0), 3)
        else:
            print("Lost left line", left)	

        if right != -1:
            rsquare = cv2.rectangle(view,
                                    (right - self.area_width, self.row_begin),
                                    (right, self.row_end),
                                    (0, 255, 0), 3)
        else:
            print("Lost right line", right)

	#self.cam_img = cv2.rectangle(self.cam_img, (0, self.roi_vertical_pos), (self.image_width - 1, self.roi_vertical_pos + self.scan_height), (255, 0, 0), 3)
	#cv2.imshow("origin", self.cam_img)
        cv2.imshow("view", view)
	print("left : " , str(left))
	print("right :" , str(right))
        cv2.waitKey(1)

    def lost_edgeline(self):
        rmid = self.image_width - self.scan_width
        pixel_cnt_threshold = 5
        # hsv = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
        #avg_value = np.average(hsv[:, :, 2])
        #value_threshold = 200
        #lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        #ubound = np.array([100, 255, 255], dtype=np.uint8)
        #self.mask = cv2.inRange(hsv, lbound, ubound)
        bin_right = -1

        for r in range(rmid, self.image_width - self.area_width):
            area = self.mask[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                bin_right = r
                break

        return bin_right
