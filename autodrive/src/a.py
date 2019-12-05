#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver

class AutoDrive:
    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
	self.angle = 0
    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l,line_r = self.line_detector.detect_lines()
        #self.line_detector.show_images(line_l,line_r)
        self.angle = self.steer(line_l,line_r)
        speed = self.accelerate(self.angle, obs_l, obs_m, obs_r)
        self.driver.drive(self.angle + 90, speed + 90)
    
    def steer(self, left, right):
        if self.angle <= 0 and left > 150:
            return self.angle
	self.angle = 0
	if left <20 : 
	    self.angle = -55
	elif left < 50:
	    self.angle = -25 
	elif left >100 : 
	    self.angle = 50
	elif left > 80:
	    self.angle = 25
	"""
	if left == -1 :
	   self.angle = -50
	if left != -1:
	    if left < 20 : 
		self.angle = -50
            elif left < 30:
                self.angle = -40
	    elif left < 60 :
	        self.angle = -15
	    elif left > 120 : 
	        self.angle = 50
            elif left > 90:
                self.angle = 25
            else:
                self.angle = 0

        else:
	    self.angle = -50
	    #if self.angle > 0 and right  :
		#return angle

	    if right == -1:
		self.angle =0
	    elif right < 510:
	        self.angle = -50
	    elif right < 540:
		self.angle = -25
	    elif right >= 610:	
		self.angle = 50
	    elif right >= 570:
	        self.angle = 25
            else : 
		self.angle = 0
	    #self.angle = -50
	"""
	return self.angle


    def accelerate(self, angle, left, mid, right):
        if min(left, mid, right) < 50:
            speed = 30
        elif angle < -39 or angle > 39:
            speed = 30
        else:
            speed = 35
        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
