#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 2018 ERC sends 2d coordinates into  ref frame
# ITU Rover Team
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib
import tf
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
string_iki_nokta= ' '
def move():
	
	global client,  string_iki_nokta
	print(" client is waiting")
	client.wait_for_server()
	print(" client is on")
	string_status=' '

	while not rospy.is_shutdown():
		if(string_status =='e '):
			break
		elif (string_status !='e'):
			string_status=' '
			print("iki nokta verelim mi    evet icin k hayir icin l")
			string_iki_nokta = raw_input()
			if(string_iki_nokta=='k'):	
				print("Enter x1  ")
				string_x1 = raw_input()
				print("Enter y1  ")
				string_y1 = raw_input()
				print("Enter yaw angle1 ")
				string_yaw1 = raw_input()
				 
			print("Enter x  ")
			string_x = raw_input()
			print("Enter y  ")
			string_y = raw_input()
			print("Enter yaw angle  ")
			string_yaw = raw_input()
			if(string_iki_nokta=='k'):
				send_move_base(float(string_x1)-4.27 , -float(string_y1)+24.29,float(string_yaw1))
			send_move_base(float(string_x)-4.27, -(float(string_y)-24.29),float(string_yaw))

			while(string_status != 'c'):
				print("Pause goal for p , Resume goal for r , abort goal for q , new goal for c , exit for e ")
				string_status= raw_input()
				if(string_status =='p'):
					print("Pausing")
					pause_move_base()
				elif(string_status=='r'):
					print("Resuming")
					send_move_base(float(string_x), float(string_y),float(string_yaw))
				elif(string_status=='q'):
					print("Aborting")
					pause_move_base()
				elif(string_status=='e'):
					print("Exiting from program..")
					break
				elif(string_status=='c'):
					break


def  send_move_base(x, y , yaw ):
	
	global client, string_iki_nokta
	goal=MoveBaseGoal()
	goal.target_pose.header.frame_id = "/odom"
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y =y
	goal.target_pose.pose.position.z = 0

	q = tf.transformations.quaternion_from_euler(0,0,yaw)
	goal.target_pose.pose.orientation.x = q[0]
	goal.target_pose.pose.orientation.y = q[1]
	goal.target_pose.pose.orientation.z = q[2]
	goal.target_pose.pose.orientation.w = q[3] 
  
	client.send_goal(goal)
	if(string_iki_nokta=='k'):
		wait = client.wait_for_result()
		string_iki_nokta=' '
	#wait = client.wait_for_result()
def  pause_move_base():
	global client
	goal=MoveBaseGoal()
	goal.target_pose.header.frame_id = "/odom"
	client.cancel_all_goals()
	client.cancel_goal()




if __name__ == '__main__':
	try:
		rospy.init_node('send_coordinates')
		move()
	except rospy.ROSInterruptException:
		rospy.loginfo("Exception thrown")

