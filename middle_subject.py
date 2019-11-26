#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from filter import MovingAverage

motor_pub = None
usonic_data = None
flag = True
count = 0


def init_node():
    global motor_pub
    rospy.init_node('sample')
    rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
    motor_pub = rospy.Publisher('xycar_motor_msg',
                                Int32MultiArray, queue_size=1)

def exit_node():
    print('finished')

def drive(angle, speed):
    global motor_pub
    drive_info = [angle, speed]
    pub_data = Int32MultiArray(data=drive_info)
    motor_pub.publish(pub_data)

def callback(data):
    global usonic_data
    usonic_data = data.data

def forward():
    global flag
    if frontsonic.get_wmm() <= 60:
        flag = False
		for i in range(50):
			drive(90, 90)
    else: 
		drive(90,110 + 10*count)

def backward():
    global flag
    if backsonic.get_wmm() <= 60:
        flag = True
		count += 1
		for i in range(50):
			drive(90, 90)
    else:
        drive(90,70)

if __name__ == '__main__':
    init_node()
    time.sleep(3)
	frontsonic = MovingAverage(5)
	backsonic = MovingAverage(5)	

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and count < 3:
		frontsonic.add_sample(usonic_data[1])
		backsonic.add_sample(usonic_data[4])
        if flag:
            forward()
		elif temp == True:
			for i in range(2):
				drive(90, 90)
				time.sleep(0.1)
				drive(90, 60)
				time.sleep(0.1)
		else:
			backward()
		temp = flag

    rospy.on_shutdown(exit_node)
