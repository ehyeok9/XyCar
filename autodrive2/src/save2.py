#!/usr/bin/env python

import rospy, time

from save1 import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver

class AutoDrive:
    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
	self.a = 0  
    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_r = self.line_detector.detect_lines()
        self.line_detector.show_images(line_r)
        angle = self.steer(line_r)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
        self.driver.drive(angle + 90, speed + 90)
    
    def steer(self, right):
	angle = 0
	if self.a == 0 and right == -1:
	    angle = 0
	elif self.a < 0 and right == -1:
	    angle -40
        elif self.a > 0 and right <= 537:
	    angle = self.a
        elif right <= 537:
            angle = -50
        elif right >= 610:
            angle = 50
        else:
            angle = 0
	self.a = angle
        return angle
    def accelerate(self, angle, left, mid, right):
        if min(left, mid, right) < 50:
            speed = 0
        elif angle < -20 or angle > 20:
            speed = 20
        else:
            speed = 30
        return 30

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
