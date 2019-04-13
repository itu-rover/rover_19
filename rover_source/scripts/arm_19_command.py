#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Subscribes /rover_arm_controller/state and /gripper_command19 topics, publishes joint states and gripper command to serial and ui.





import rospy
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_msgs.msg import Int8

import math


class joint_states(object):
	def __init__(self):



		rospy.init_node("arm_19_command_publisher")
		rospy.Subscriber("/rover_arm_controller/state", JointTrajectoryControllerState, self.arm_callback)
		rospy.Subscriber("/gripper_command19", Int8, self.gripper_callback)
		
		self.pub = rospy.Publisher("/arm_19_serial", String, queue_size = 50)
		self.pub_ui = rospy.Publisher("/arm_19_ui", String, queue_size = 50)

		#self.joint_commands = JointTrajectoryControllerState()

		#self.joint_commands.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
		#self.joint_states.position = [1, 1, 1, 1, 1, 1]
		#self.joint_states.effort = [10, 10, 10, 10, 10, 10]
		self.joint1_str = "000"
		self.joint2_str = "000"           
		self.joint3_str = "000"         #These values must be set to minimum joint_str
		self.joint4_str = "000"
		self.joint5_str = "000"
		self.joint6_str = "000"

		self.gripper_command = "2"

		self.rate = rospy.Rate(30)


		while not rospy.is_shutdown():
			
		
			self.command_publisher()
			#print(self.joint_commands)

			self.rate.sleep()

			

	def arm_callback(self, command) :

		#command = data.data

		joint1 = command.desired.positions[0]  #Change positions to velocities for velocity control
		joint2 = command.desired.positions[1]
		joint3 = command.desired.positions[2]
		joint4 = command.desired.positions[3]
		joint5 = command.desired.positions[4]
		joint6 = command.desired.positions[5]

		print("joint1: " + str(joint1))
		print("joint2: " + str(joint2))
		print("joint3: " + str(joint3))
		print("joint4: " + str(joint4))
		print("joint5: " + str(joint5))
		print("joint6: " + str(joint6) + "\n")

				

		"""joint1 = elk_to_rad(joint1_int, 0, 1008, 90, -90)
		joint2 = elk_to_rad(joint2_int, 460, 675, 84, 15)
		joint3 = elk_to_rad(joint3_int,710, 920, 104, 55 )
		joint4 = elk_to_rad(joint4_int, 1008, 20, -90, 180)
		joint5 = elk_to_rad(joint5_int, 870, 550, 90, 0)
		joint6 = elk_to_rad(joint6_int, 0, 1008, 90, -90)"""
		
		joint1_elk = rad_to_elk(joint1, -3.14, 3.14,   0, 831)   #This part needs to be updated according to the mapping used by electronics. 
		joint2_elk = rad_to_elk(joint2, -3.14, 3.14,   0, 831)   #Boundaries of degree values must be found by using Moveit Setup Assistant. 
		joint3_elk = rad_to_elk(joint3, -3.14, 3.14,   0, 831)
		joint4_elk = rad_to_elk(joint4, -3.14, 3.14,   0, 831)
		joint5_elk = rad_to_elk(joint5, -3.14, 3.14,   0, 831)
		joint6_elk = rad_to_elk(joint6, -3.14, 3.14,   0, 831)

		self.joint1_str = floattostring(joint1_elk)
		self.joint2_str = floattostring(joint2_elk)
		self.joint3_str = floattostring(joint3_elk)
		self.joint4_str = floattostring(joint4_elk)
		self.joint5_str = floattostring(joint5_elk)
		self.joint6_str = floattostring(joint6_elk)


	def gripper_callback(self, data) :

		self.gripper_command = str(data.data)

		

	

	def command_publisher(self) :

		self.pub.publish("S"+self.joint1_str+self.joint2_str+self.joint3_str+self.joint4_str+self.joint5_str+self.joint6_str+"CF")
		self.pub_ui.publish(self.joint1_str+" "+self.joint2_str+" "+self.joint3_str+" "+self.joint4_str+" "+self.joint5_str+" "+self.joint6_str+" "+self.gripper_command)


def rad_to_elk(joint_rad, rad_min, rad_max, elk_min, elk_max) :              #Converts joint states from rad to elk unit and constrains the result

	joint_elk = (joint_rad - rad_min)*(elk_max - elk_min)/(rad_max - rad_min) + elk_min

	if joint_elk < elk_min :

		joint_elk = elk_min

	if joint_elk > elk_max :

		joint_elk = elk_max

	return joint_elk

def floattostring(joint):

	value = int(joint)
	
	if value < 10 :
		
		string = "00" + str(value)
	
	elif value < 100 and value > 9 :
		
		string = "0" + str(value)
	
	else:
		
		string = str(value)
	
	return string



if __name__ == '__main__':
	joint_states()
