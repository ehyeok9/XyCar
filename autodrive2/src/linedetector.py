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

        gray = cv2.cvtColor(self.roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)

    def trafficLight(self):
        screen = self.cam_img
        roi =screen[120:370, 470:640]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
	#cv2.imshow('1', roi)
	#cv2.waitKey(1)
        scan_height, scan_width = 250,170
        pixel_value = 0.1*scan_width*scan_height


        lower_red = np.array([0, 0, 210], dtype=np.uint8)
        upper_red = np.array([130, 255,255], dtype=np.uint8)
        red_range = cv2.inRange(hsv, lower_red, upper_red)
	#cv2.imshow("1", red_range)
	cv2.waitKey(1)
        red_value = cv2.countNonZero(red_range)

        return_value = 0

        if (red_value > pixel_value):
            return_value = 1
	
        return return_value
	

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
	#print("left : " ,str(left)) 
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
#	print("left : " , str(left))
#	print("right :" , str(right))
        cv2.waitKey(1)

