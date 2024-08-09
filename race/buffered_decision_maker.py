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

# 설정 가능한 버퍼 크기 변수
BUFFER_SIZE = 10  # 예측 지연 시간을 결정하는 버퍼 크기

bridge = CvBridge()
model = tf.keras.models.load_model("/home/fmtc/catkin_ws/src/race/scripts/dowellmodel.h5")
frame = None
control = deque(maxlen=5)
prediction_buffer = deque(maxlen=BUFFER_SIZE)

def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # Change Image_msg to image_array(cv2)

def control_choose(prediction):  # weighted average for recent 5 frames
    control.append(prediction)
    if len(control) > 4:
        mean_val = (5*control[4] + 4*control[3] + 3*control[2] + 2*control[1] + control[0])/15
        if mean_val < 2:  # linear interpolation: left_max = 15, middle = 0, right_max = -12
            steer_val = round(-22*mean_val + 44)
        else:
            steer_val = round(-15*mean_val + 30)
        
        control.popleft()  # 가장 오래된 항목을 제거

        return steer_val
    else:
        return prediction  # return just prediction if frames < 5

def Steer_publisher():
    rospy.init_node('decision_maker', anonymous=True)  # init node
    pub = rospy.Publisher('Steer_value', Int16, queue_size=10)  # define topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)  # get image from usb_cam node
    rate = rospy.Rate(32)

    while not rospy.is_shutdown():
        if frame is not None:
            reshaped_frame = frame.reshape(-1, 480, 640, 3)
            prediction = int(tf.argmax(model(reshaped_frame), axis=1)) + 1
            prediction_buffer.append(prediction)

            if len(prediction_buffer) == BUFFER_SIZE:
                buffered_prediction = prediction_buffer.popleft()
                current_control = control_choose(buffered_prediction)
                pub.publish(current_control)  # publish steer_value to arduino
                rospy.loginfo(f'Predicted class: {current_control}')
            else:
                # 버퍼가 다 차기 전에는 가장 최근 예측 값을 사용
                current_control = control_choose(prediction)
                pub.publish(current_control)  # publish steer_value to arduino
                rospy.loginfo(f'Using recent prediction: {current_control}')

        rate.sleep()

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        Steer_publisher()
    except rospy.ROSInterruptException:
        pass

