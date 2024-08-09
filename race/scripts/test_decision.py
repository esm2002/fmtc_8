#!/usr/bin/env python3

'''
DecisionMaker_for_race.py

1. 카메라에서 받은 정보를 바탕으로 자율주행을 판단하는 코드
2. usb_cam 노드에서 publish한 topic(image_raw/compressed)을 subscribe
3. 딥러닝을 통해 image로부터 조향값(가변저항)을 계산
5. 아두이노(serial_node)에 최종 명령(steer_value) publish

'''

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
import tensorflow as tf
from collections import deque

bridge = CvBridge()
model = tf.keras.models.load_model("/home/fmtc/catkin_ws/src/race/scripts/plzdowell.h5")
frame = None
control = deque(maxlen=5)

def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # Change Image_msg to image_array(cv2)
    frame = cv2.resize(frame, (320, 240))  # 이미지 크기 조정

def control_choose(prediction):  # weighted average for recent 5 frames
    control.append(prediction)
    if len(control) == 5:
        mean_val = sum(weight * control[-(i + 1)] for i, weight in enumerate([5, 4, 3, 2, 1])) / 15
        if mean_val < 2:  # linear interpolation: left_max = 15, middle = 0, right_max = -12
            steer_val = round(-22 * mean_val + 44)
        else:
            steer_val = round(-15 * mean_val + 30)
        return steer_val
    return prediction

def Steer_publisher():
    rospy.init_node('decision_maker', anonymous=True)  # init node
    pub = rospy.Publisher('Steer_value', Int16, queue_size=10)  # define topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)  # get image from usb_cam node
    rate = rospy.Rate(32)

    while not rospy.is_shutdown():
        if frame is not None:
            reshaped_frame = np.expand_dims(frame, axis=0)  # 모델 입력 형태로 변환 (1, 240, 320, 3)
            if reshaped_frame.shape == (1, 240, 320, 3):
                prediction = Int16()
                prediction.data = int(tf.argmax(model(reshaped_frame), axis=1)) + 1  # decide direction from image by CNN model
                # model outputs are 0 for left, 1 for middle, 2 for right. +1 to make them (1, 2, 3)
                current_control = control_choose(prediction.data)  # post processing, calculate steer_value
                pub.publish(current_control)  # publish steer_value to arduino
                rospy.loginfo(f'Predicted class: {current_control}')
            else:
                rospy.logwarn(f'Unexpected frame shape: {reshaped_frame.shape}')

        rate.sleep()

if __name__ == "__main__":
    try:
        Steer_publisher()
    except rospy.ROSInterruptException:
        pass

