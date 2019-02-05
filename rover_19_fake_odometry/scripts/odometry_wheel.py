#!/usr/bin/env python

import rospy
from nav_msgs import Odometry

if __name__ == '__main__':

	rospy.init_node('odometry_wheel_publisher')

	pub = rospy.Publisher("/number", Odometry, queue_size=10)

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():

		msg = Odometry()

		msg.data = 2

		pub.publish(msg)

		rate.sleep()


