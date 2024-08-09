#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import pickle
import cv2
from cv_bridge import CvBridge
import time

def image_callback(msg):
    global frame
    # Convert compressed image message to OpenCV format
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

def image_saver():
    # Initialize the ROS node
    rospy.init_node('image_saver', anonymous=True)

    # Subscribe to the compressed image topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)

    image_list = []

    print("Press 'Ctrl+C' to quit")

    save_interval = 1  # Save an image every second
    rate = rospy.Rate(1 / save_interval)  # Define the rate for saving images

    try:
        while not rospy.is_shutdown():
            if frame is not None:
                image_list.append(frame)
                print(f"Image saved, total images: {len(image_list)}")

            rate.sleep()
    except KeyboardInterrupt:
        file_path = "traf.pickle"
        with open(file_path, "wb") as file:
            pickle.dump(image_list, file)
        print(f"Saved {len(image_list)} images to {file_path}")

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        image_saver()
    except rospy.ROSInterruptException:
        pass


