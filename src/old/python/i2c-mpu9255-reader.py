#!/usr/bin/env python
import rospy
import smbus
import numpy as np
from std_msgs.msg import Int16MultiArray

MPU9255 = smbus.SMBus(1)
MPU9255_ADDRESS = 0x68

pub = rospy.Publisher('/mpu9255_raw_data', Int16MultiArray)
rospy.init_node('mpu9255_reader', anonymous = True)

imu_data = Int16MultiArray()
imu_data.data = [0, 0, 0, 0, 0, 0]

while not rospy.is_shutdown():
	accel_data = np.int8(MPU9255.read_i2c_block_data(MPU9255_ADDRESS, 0x3B, 6))
	gyro_data = np.int8(MPU9255.read_i2c_block_data(MPU9255_ADDRESS, 0x43, 6))

	imu_data.data[0] = (accel_data[0] << 8) + accel_data[1]
	imu_data.data[1] = (accel_data[2] << 8) + accel_data[3]
	imu_data.data[2] = (accel_data[4] << 8) + accel_data[5]
	imu_data.data[3] = (gyro_data[0] << 8) + gyro_data[1]
	imu_data.data[4] = (gyro_data[2] << 8) + gyro_data[3]
	imu_data.data[5] = (gyro_data[4] << 8) + gyro_data[5]

	try:
		pub.publish(imu_data)
	except rospy.exceptions.ROSSerializationException, e:
		print e
