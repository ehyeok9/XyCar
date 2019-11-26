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

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l, line_r = self.line_detector.detect_lines()
        self.line_detector.show_images(line_l, line_r)
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
        self.driver.drive(angle + 90, speed + 90)
    
    def steer(self, left):
        # mid = (left + right) // 2

        # if mid < 280:
        #     angle = int((320 - mid) * 0.25)
        # elif mid > 360:
        #     angle = -int((mid - 320) * 0.25)
        # else:
        #     angle = 0

        # if left == -1 and right == -1:
        #     angle = -1
        # elif left == -1:
        #     angle = -40
        # elif right == -1:
        #     angle = 40
        angle = 0

        if left < 0:
            angle = -20
        elif left < 50:
            angle = -40
        elif left > 120:
            angle = 40

        return angle
    
    def accelerate(self, angle, left, mid, right):

        if min(left, mid, right) < 50:
            speed = 0
        elif angle < -20 or angle > 20:
            speed = 20
        else:
            speed = 30

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

