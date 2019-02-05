#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rover_source.msg import *

joints =Joint_msgs()

joint_state_vector=[joints.joint1, joints.joint2, joints.joint3, joints.joint4, joints.joint5, joints.joint6]


def callback(data,self):
    self.splitted=data.data.split(',')

    if (self.splitted[0]=='S'):
        if(float(self.splitted[1])>=1000):
            joints.joint1=(float(self.splitted[1])-1000)
	    if(float(self.splitted[1])<1000):
	        joints.joint1=(-float(self.splitted[1]))

        if(float(self.splitted[2])>=1000):
	        joints.joint2=(float(self.splitted[2])-1000)
	    if(float(self.splitted[2])<1000):
	        joints.joint2=(-float(self.splitted[2]))

	    if(float(self.splitted[3])>=1000):
            joints.joint3=(float(self.splitted[3])-1000)
	    if(float(self.splitted[3])<1000):
	        joints.joint3=(-float(self.splitted[3]) )

	    if(float(self.splitted[4])>=1000):
            joints.joint4=(float(self.splitted[4])-1000)
	    if(float(self.splitted[4])<1000):
            joints.joint4=(-float(self.splitted[4]) )

        if(float(self.splitted[5])>=1000):
	        joints.joint5=(float(self.splitted[4])-1000)
        if(float(self.splitted[5])<1000):
            joints.joint5=(-float(self.splitted[4]) )

        if(float(self.splitted[6])>=1000):
	        joints.joint6=(float(self.splitted[4])-1000)
        if(float(self.splitted[6])<1000):
            joints.joint6=(-float(self.splitted[4]) )

def publisher():

    while not rospy.is_shutdown():

        pub.publish(joint_state_vector)


def main():
    rospy.init_node('rover_joint_state_publisher')
	pub = rospy.Publisher('/rover_joint_state_publisher', Joint_msgs, queue_size = 50)
    rospy.Subscriber('/rover_serial_encoder', String, callback)
    publisher()
