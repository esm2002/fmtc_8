#!/usr/bin/env python3

import rospy
import rosnode
from std_msgs.msg import Int16
#from lidar.msg import parking_detection

from queue import Queue


class ControlCommand() : 
    
    def __init__(self) : 
        
        rospy.init_node('parking_decision', anonymous=True)

        rospy.Subscriber('parking_flag',Int16, self.lidar_callback)
        
        #self.parking_value = parking_detection()
        self.lidar_obstacle_detect = 0
        #self.lidar_obstacle_y_coord = 0 
        
            
    
    def lidar_callback(self, msg):
        
        self.lidar_obstacle_detect = msg.data
        #self.lidar_obstacle_y_coord = msg.y_coord
        
    def arduino_command_pub(self):
        
        self.command_pub = rospy.Publisher('parking_val', Int16, queue_size=3)
        
        parking_msg = Int16()

        print("주차 중입니다. flag: ", self.lidar_obstacle_detect)

        parking_msg.data = self.lidar_obstacle_detect


        self.command_pub.publish(parking_msg)

        
    def run(self) : 
        
        # rate = rospy.Rate(30)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
                
           self.arduino_command_pub() 
           rate.sleep()


if __name__ == '__main__':
    try:
        Control_Command = ControlCommand()
        Control_Command.run()
        
    except rospy.ROSInterruptException:
        pass
