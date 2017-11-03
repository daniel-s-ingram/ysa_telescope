#!/usr/bin/env python

import rospy
import pigpio
from std_msgs.msg import Float32MultiArray

pi = pigpio.pi()

def callback(data):
	roll = data.data[0]
        pitch = data.data[1]
	roll_servo = int(555.6 * angle + 50000)
	pitch_servo = int(555.6 * otherangle + 50000)	

	pi.hardware_PWM(12, 50, roll_servo)
        pi.hardware_PWM(13, 50, pitch_servo)

rospy.init_node('pid_servo_controller', anonymous = True)
rospy.Subscriber('/orientation', Float32MultiArray, callback)

rospy.spin()

