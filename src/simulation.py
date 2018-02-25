#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Float64

arduino = serial.Serial('/dev/ttyUSB0', 115200)

rospy.init_node('sim_joint_controller', anonymous=True)

roll_pub = rospy.Publisher('/ysa/roll_joint_controller/command', Float64, queue_size=10)
pitch_pub = rospy.Publisher('/ysa/pitch_joint_controller/command', Float64, queue_size=10)
yaw_pub = rospy.Publisher('/ysa/yaw_joint_controller/command', Float64, queue_size=10)

roll_msg = Float64()
pitch_msg = Float64()
yaw_msg = Float64()

while not rospy.is_shutdown():
	rpy = arduino.readline().split()

	try:
		roll_msg.data = float(rpy[0])
		pitch_msg.data = float(rpy[1])
	except ValueError:
		continue

	roll_pub.publish(roll_msg)
	pitch_pub.publish(pitch_msg)

arduino.close()