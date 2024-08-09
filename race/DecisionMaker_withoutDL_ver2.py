#!/usr/bin/env python3

'''
DecisionMaker_for_race_withoutDL.py

1. 카메라에서 받은 정보를 바탕으로 자율주행을 판단하는 코드
2. usb_cam 노드에서 publish한 topic(image_raw/compressed)을 subscribe
3. 이미지 처리 기법을 통해 조향값(가변저항)을 계산
5. 아두이노(serial_node)에 최종 명령(steer_value) publish
'''

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import deque

control = deque(maxlen=5)

def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # Change Imgage_msg to image_array(cv2)

def process_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(binary, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50)

    if lines is not None:
        left_lines = [line for line in lines if line[0][0] < image.shape[1] / 2]
        right_lines = [line for line in lines if line[0][0] >= image.shape[1] / 2]

        if left_lines and right_lines:
            left_avg = np.mean([line[0][0] for line in left_lines])
            right_avg = np.mean([line[0][0] for line in right_lines])

            lane_center = (left_avg + right_avg) / 2
            image_center = image.shape[1] / 2

            deviation = lane_center - image_center
            steer_value = int(np.clip(deviation / (image.shape[1] / 2) * 15, -15, 15))

            return steer_value
    return 0

def control_choose(prediction):
    control.append(prediction)
    if len(control) >= 5:
        mean_val = sum(control) / len(control)
        return int(mean_val)
    else:
        return prediction

def Steer_publisher():
    rospy.init_node('decision_maker', anonymous=True) # init node
    pub = rospy.Publisher('Steer_value', Int16, queue_size=10) # define topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback) # get image from usb_cam node
    rate = rospy.Rate(32)

    while not rospy.is_shutdown():
        if frame is not None:
            steer_value = process_image(frame)
            current_control = control_choose(steer_value)
            pub.publish(current_control)
            rospy.loginfo(f'Predicted steer value: {current_control}')
        rate.sleep()

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        Steer_publisher()
    except rospy.ROSInterruptException:
        pass

