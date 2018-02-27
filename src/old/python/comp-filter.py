#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

orientation = Float32MultiArray()
orientation.data = [0, 0]
a = 0.98

def callback(data):
	global orientation, a, t
	orientation.data[0] = a * (orientation.data[0] - data.data[2] * (rospy.get_time() - t)) + (1 - a) * data.data[0]
        orientation.data[1] = a * (orientation.data[1] + data.data[3] * (rospy.get_time() - t)) + (1 - a) * data.data[1]
	t = rospy.get_time()

	pub.publish(orientation)

rospy.init_node('kalman_filter', anonymous = True)

t = rospy.get_time()

rospy.Subscriber('/orientation_info', Float32MultiArray, callback)
pub = rospy.Publisher('/orientation', Float32MultiArray)

rospy.spin()

