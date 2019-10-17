import rospy, time
from std_msgs.msg import Int32MultiArray

motor_pub = None
usonic_data = None
flag = True

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
    if data[1] <= 60:
        for i in range(2):
            drive(90,90)
            time.sleep(0.1)
        flag = False
    else: drive(90,120)

def backward():
    global flag
    if data[5] <= 60:
        drive(90,90)
        flag = True
    else:
        drive(90,60)

if __name__ == '__main__':
    init_node()
    time.sleep(3)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if flag:
            forward()
        else:
            backward()

    rospy.on_shutdown(exit_node)
