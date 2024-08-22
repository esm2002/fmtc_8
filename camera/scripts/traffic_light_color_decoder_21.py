#!/usr/bin/env python3

"""
traffic_light_color_decoder_final.py
1. 신호등 색 판단해 stop_flag 판단 및 bbox_size 계산 
2. yolo_final.py에서 publish한 topic(color, bbox_info)를 subscribe
3. test_traffic_light.py에 stop_flag, bbox_size msg publish

"""
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Int32MultiArray
import time
from time import strftime, localtime

from camera.msg import traffic_light_stop # traffic_light_stop.msg 파일 참고(custom msg) : stop_flag, bbox_size
from camera.msg import traffic_light_info # traffic_light_info.msg 파일 참고(custom msg) : color, bbox_info

class TrafficLightDecoder: 

    def __init__(self):

        rospy.init_node("traffic_node", anonymous=True) # traffic_node라는 이름으로 노드 초기화
        self.bridge = CvBridge() 
        
        self.cap_img = []
        self.bbox = []
        self.traffic_light_color = "nothing"

        # ***** 수정한 부분, 스택 초기화
        self.stop_flag_stack = [0] * 6

        # yolo_final.py에서 publish한 topic(color, bbox_info)를 subscribe
        self._bbox_sub = rospy.Subscriber('/vision/bbox/traffic_light', traffic_light_info, self._bbox_callback)

        # test_traffic_decision.py에 (stop_flag, bbox_size) publish
        self.traffic_flag_pub = rospy.Publisher("stop_signal", traffic_light_stop, queue_size=2) 
        

    def _bbox_callback(self, msg): # yolo_final.py에서 publish한 topic(color, bbox_info)를 subscribe하기 위한 callback 함수
        
        self.traffic_light_color = msg.color
        self.bbox = msg.bbox_info


    def decode_traffic_signal(self): # 신호등 색 판단하여 stop_flag, bbox_size return
        
        print(self.traffic_light_color)
        print(self.bbox)
        
        if len(self.bbox) == 0:  # bbox가 없으면 : 감지된 신호등이 없으면
            print("[Traffic Light Decoder] No traffic light detected")
            # stop_flag = 0
            # bbox_size = 0
            return 0, 0
        
        else : # bbox가 있으면 : 감지된 신호등이 있으면

            print("[Traffic Light Decoder] traffic light detected")
            [xmin, ymin, xmax, ymax] = self.bbox
            bbox_size = (xmax-xmin) * (ymax-ymin) # bbox의 크기 계산
            print(bbox_size)
            
            print(self.traffic_light_color, " color I received from yolo")
            
            if(self.traffic_light_color == 'red'): # 감지된 신호등이 red이면
                print("Traffic Light RED detected")
                stop_flag = 1 # 정지신호 전달
            
            if(self.traffic_light_color == 'green'):
                print("Traffic Light GREEN detected")
                stop_flag = 0

            # 어떤 신호를 감지하였으나 초록, 빨강 모두가 아닌 경우
            if(self.traffic_light_color != 'red' and self.traffic_light_color != 'green'):
                print("bbox detected, but no signal detected")
                print("Previous stop_sign is used again")
                # 기존 신호를 그대로 다시 쓰도록
                stop_flag = self.stop_flag_stack[-1]

            return stop_flag, bbox_size
        
        
    def traffic_flag_publish(self): # stop_flag, bbox_size publish
        
        msg = traffic_light_stop()  #taffic_light_stop.msg 파일 참고 : stop_flag, bbox_size
        
        result = self.decode_traffic_signal() # stop_flag, bbox_size 

        # ****** 전체적으로 수정함
        if result is not None:  # 감지된 신호등이 있으면
            stop_flag, bbox_size = result
            msg.bbox_size = bbox_size

            # 스택 업데이트 (가장 오래된 값 제거 후 새 값 추가)
            self.stop_flag_stack.pop(0)  # 가장 오래된 값 제거
            self.stop_flag_stack.append(stop_flag)  # 새 값 추가

            # 빨간 불이 감지되면 즉시 멈춤
            if stop_flag == 1:
                msg.stop_flag = 1
                self.traffic_flag_pub.publish(msg)
                return  # 함수 종료

            # 스택의 평균값 계산
            average_stop_flag = np.mean(self.stop_flag_stack)
            print(f"Average Stop Flag: {average_stop_flag}")

            # 평균이 0.34 이하이면 멈추지 않음
            if average_stop_flag <= 0.34:
                stop_flag = 0

            # 메시지에 최종 stop_flag 설정
            msg.stop_flag = stop_flag

        else:  # 감지된 신호등이 없으면
            msg.stop_flag = 0
            msg.bbox_size = 0
        
        self.traffic_flag_pub.publish(msg)


if __name__ == "__main__":
    traffic_decoder = TrafficLightDecoder()
    rospy.sleep(1)  # wait for node initialization

    rate = rospy.Rate(10)  # FIXME: check the rate

    while not rospy.is_shutdown():
        print("\n==========================================================\n")
        inf_start = time.time()

        # publish stop flag
        traffic_decoder.traffic_flag_publish()

        print("TL judging time : ", (time.time() - inf_start) * 1000, "ms") # 판단하는데 걸린 시간
        print("\n==========================================================\n")

        rate.sleep()


# 스택 평균을 수정한다면 6회에 2회 오류까지 허용 (?) 그러면 "*5 -> *6" 그리고 "<=0.2 -> <=0.34" 이건 나중에 한 번 봐야할 듯
