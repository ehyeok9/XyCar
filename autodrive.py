#!/usr/bin/env python

# https://github.com/HyOsori/Osori-SelfDrivingWithGTA5/wiki/%EA%B0%95%EC%A2%8C-7----OpenCV%EB%A5%BC-%EC%9D%B4%EC%9A%A9%ED%95%9C-%EC%9E%90%EC%9C%A8-%EC%A3%BC%ED%96%89-%EC%86%8C%EA%B0%9C

import rospy
import time
import numpy as np
from PIL import ImageGrab
import cv2
from numpy import ones, vstack
from numpy.linalg import lstsq
from statistics import mean

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver2')
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

    def draw_lanes(img, lines, color=[0, 255, 255], thickness=3):

        try:
            ys = []
            for i in lines:
                for ii in i:
                    ys += [ii[1], ii[3]]
            min_y = min(ys)
            max_y = 600
            new_lines = []
            line_dict = {}

            for idx, i in enumerate(lines):
                for xyxy in i:
                    x_coords = (xyxy[0], xyxy[2])
                    y_coords = (xyxy[1], xyxy[3])
                    A = vstack([x_coords, ones(len(x_coords))]).T
                    m, b = lstsq(A, y_coords)[0]

                    # Calculating our new, and improved, xs
                    x1 = (min_y - b) / m
                    x2 = (max_y - b) / m

                    line_dict[idx] = [m, b, [int(x1), min_y, int(x2), max_y]]
                    new_lines.append([int(x1), min_y, int(x2), max_y])

            final_lanes = {}

            for idx in line_dict:
                final_lanes_copy = final_lanes.copy()
                m = line_dict[idx][0]
                b = line_dict[idx][1]
                line = line_dict[idx][2]

                if len(final_lanes) == 0:
                    final_lanes[m] = [[m, b, line]]

                else:
                    found_copy = False

                    for other_ms in final_lanes_copy:

                        if not found_copy:
                            if abs(other_ms * 1.2) > abs(m) > abs(other_ms * 0.8):
                                if abs(final_lanes_copy[other_ms][0][1] * 1.2) > abs(b) > abs(
                                        final_lanes_copy[other_ms][0][1] * 0.8):
                                    final_lanes[other_ms].append([m, b, line])
                                    found_copy = True
                                    break
                            else:
                                final_lanes[m] = [[m, b, line]]

            line_counter = {}

            for lanes in final_lanes:
                line_counter[lanes] = len(final_lanes[lanes])

            top_lanes = sorted(line_counter.items(), key=lambda item: item[1])[::-1][:2]

            lane1_id = top_lanes[0][0]
            lane2_id = top_lanes[1][0]

            def average_lane(lane_data):
                x1s = []
                y1s = []
                x2s = []
                y2s = []
                for data in lane_data:
                    x1s.append(data[2][0])
                    y1s.append(data[2][1])
                    x2s.append(data[2][2])
                    y2s.append(data[2][3])
                return int(mean(x1s)), int(mean(y1s)), int(mean(x2s)), int(mean(y2s))

            l1_x1, l1_y1, l1_x2, l1_y2 = average_lane(final_lanes[lane1_id])
            l2_x1, l2_y1, l2_x2, l2_y2 = average_lane(final_lanes[lane2_id])

            return [l1_x1, l1_y1, l1_x2, l1_y2], [l2_x1, l2_y1, l2_x2, l2_y2], lane1_id, lane2_id
        except Exception as e:
            print(str(e))

        def process_img(image):
            original_image = image

            # RGB to GRAY
            processed_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Canny 외곽선 검출
            processed_img = cv2.Canny(processed_img, threshold1=200, threshold2=300)

            # 이미지 가우시안 정규화
            processed_img = cv2.GaussianBlur(processed_img, (5, 5), 0)

            # 관심영역만 필터링, 4강 참고
            vertices = np.array([[10, 500], [10, 300], [300, 200], [500, 200], [800, 300], [800, 500], ], np.int32)
            processed_img = roi(processed_img, [vertices])

            # Hough 알고리즘, 5강 참고
            lines = cv2.HoughLinesP(processed_img, 1, np.pi / 180, 180, 20, 15)

            m1 = 0  # 차선 1의 기울기
            m2 = 0  # 차선 2의 기울기

            # 차선 그리기
            try:
                l1, l2, m1, m2 = draw_lanes(original_image, lines)
                cv2.line(original_image, (l1[0], l1[1]), (l1[2], l1[3]), [0, 255, 0], 30)
                cv2.line(original_image, (l2[0], l2[1]), (l2[2], l2[3]), [0, 255, 0], 30)

            except Exception as e:
                print(str(e))
                pass

            # 외곽선 그리기(For Debug)
            try:
                for coords in lines:
                    coords = coords[0]
                    try:
                        cv2.line(processed_img, (coords[0], coords[1]), (coords[2], coords[3]), [255, 0, 0], 3)

                    except Exception as e:
                        print(str(e))
            except Exception as e:
                pass

            return processed_img, original_image, m1, m2

        def main():
            last_time = time.time()
            while True:
                screen = np.array(ImageGrab.grab(bbox=(0, 40, 800, 640)))
                print('Frame took {} seconds'.format(time.time() - last_time))
                last_time = time.time()
                new_screen, original_image, m1, m2 = process_img(screen)
                # cv2.imshow('window', new_screen)
                cv2.imshow('window2', cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB))

                if m1 < 0 and m2 < 0:  # / /
                    right()
                elif m1 > 0 and m2 > 0:  # \ \
                    left()
                else:  # / \
                    straight()

                # cv2.imshow('window',cv2.cvtColor(screen, cv2.COLOR_BGR2RGB))
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
