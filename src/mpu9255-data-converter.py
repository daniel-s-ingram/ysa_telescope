#!/usr/bin/env python

import rospy
from math import *
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

orientation_info = Float32MultiArray()

def callback(data):
  accel_x = data.data[0]
  accel_y = data.data[1]
  accel_z = data.data[2]
  
  gyro_x = data.data[3]
  gyro_y = data.data[4]
  gyro_z = data.data[5]

  pitch = (atan2(accel_x, sqrt(accel_y ** 2 + accel_z ** 2))) * 180 / pi
  roll = (atan2(accel_y, sqrt(accel_x ** 2 + accel_z ** 2))) * 180 / pi
  pitch_rate = gyro_y * 250 / 32768
  roll_rate = gyro_x * 250 / 32768

  global orientation_info
  orientation_info.data = [pitch, roll, pitch_rate, roll_rate]

  pub.publish(orientation_info)


rospy.init_node('mpu9255_data_converter', anonymous = True)
rospy.Subscriber('/mpu9255_raw_data', Int16MultiArray, callback)
pub = rospy.Publisher('/orientation_info', Float32MultiArray)

rospy.spin()

