#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
from imuread import ImuRead

class AutoDrive:
    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.imu = ImuRead('/diagnostics')

	self.angle = 0
	self.speed = 0
    def trace(self):
	redSignal = self.line_detector.trafficLight()
	print(redSignal)
	if redSignal:
	    print(redSignal)
	    self.forward()
	r,p,y = self.imu.get_data()
        obs_l, obs_m, obs_r, obs_sl, obs_sr, B = self.obstacle_detector.get_distance()
        line_l,line_r = self.line_detector.detect_lines()
        #self.line_detector.show_images(line_l,line_r)
        self.angle = self.steer(line_l,line_r)
        speed = self.accelerate(self.angle, obs_l, obs_m, obs_r, p, obs_sl, obs_sr, B)
	self.driver.drive(self.angle + 90, speed + 90)
	#print(p)    	
	if(p<-5):
	    time.sleep(3)
    def steer(self, left, right):
	
	
        if self.angle < 0 and (100<=left<=150):
	    return self.angle

	self.angle = 0

	if left == -1 :
	    self.angle = -55

	elif left < 17 : 
	    self.angle = -48
	elif 17 <= left < 50:
	    self.angle = -19


	elif left >130 : 
	    self.angle = 47
	elif 130 >= left > 100:
	    self.angle = 18
	#print(left)
	return self.angle


    def accelerate(self, angle, left, mid, right, p, side_left, side_right, B):
	
	if mid < 40 and side_left < 40 and side_right < 40:
            self.backward()

        if p<-5:
	    return 0

        if self.speed > 0 and mid==0:
	    return self.speed
        elif (angle <= -25 or angle >= 35):
            self.speed = 37
        else:
            self.speed = 46
	#print(mid)
        return 25

    def exit(self):
        print('finished')

    def backward(self):
	for i in range(2):
            self.driver.drive(90, 90)
	    time.sleep(0.1)
	    self.driver.drive(90, 60)
            time.sleep(0.1)
        for i in range(50):
            self.driver.drive(35, 70)
            time.sleep(0.1)

    def forward(self):
	for i in range(30):
	    self.driver.drive(35, 120)
	    time.sleep(0.1)

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
