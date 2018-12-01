#!/usr/bin/env python

import rospy

from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


def callback(data):
   array = Float32MultiArray()
   array.layout.dim.append(MultiArrayDimension())
   array.layout.dim[0].label = "wheels_speed"
   array.layout.dim[0].size = 2
   array.layout.dim[0].stride = 1
   array.layout.data_offset = 0
   array.data = [0]*2

   wl = data.data[0]
   wr = data.data[1]


   rospy.loginfo("Right wheel speed %f rad/s\n\r",wr)
   rospy.loginfo("Left wheel speed %f rad/s\n\r",wl)
   


def start():
   
   rospy.Subscriber("wheels_speed", Float32MultiArray, callback)
   
   rospy.init_node('view_wheels_speed')

   rate = rospy.Rate(100)

   #while not rospy.is_shutdown():
   #   rate.sleep()
   rospy.spin()

if __name__ == '__main__':
        start()
