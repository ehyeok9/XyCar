import numpy as np
import cv2

vid = cv2.VideoCapture('1.avi')

pixel_cnt_threshold = 0.6 * 170 * 250
while True:
    ret, frame = vid.read()
    if not ret:
        break

    if cv2.waitKey(1) & 0xFF == 27:
        break
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.rectangle(frame, (470, 120), (640, 370), (0, 255, 0), 3)
    redStack = 0
    for i in range(470, 640):
       for j in range(120, 370):
            if 0 <= frame[j,i][0] <= 20 or 160 <= frame[j,i][0] <= 180 :
                redStack += 1
    if redStack >= pixel_cnt_threshold:
        print("ok")
    else:
        print("none")

    cv2.imshow('video', frame)
    cv2.waitKey(1)


vid.release()
cv2.destroyAllWindows()