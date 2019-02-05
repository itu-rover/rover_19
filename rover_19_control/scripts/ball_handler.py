#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 2018 URC  ball handler node starts when image procces finds ball!!
# bearing_to_ball  topic is a string that gives ball  angle from rover. 
# while bearing to ball is coming from positive or negative angle it will rotate the rover with that data.
# if ball's bearing angle is less than -5..+5 degree, move base sends 3 times go  forward msgs.After going 3 times, it will  write "succesfull".
# ITU Rover Team
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Twist
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf
from rover_state_mach.msg import RoverStateMsg

class GoForwardAvoid():
	def __init__(self):
		rospy.init_node('ball_handler', anonymous=False)
		self.currPosX=0
		self.currPosY=0
		self.currPosZ=0
		self.yaw=0
		self.bearingToball=0.0
		self.bearingToball_old=0.0
		self.counter=0
		self.send_once=1
		self.ball_is_found=0
	   
		self.msg="-"
		self.state=RoverStateMsg()
		self.move_msg=MoveBaseActionResult()
		self.twist = Twist()
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		print("waiting client server")
		self.client.wait_for_server()
		print(" client is on")
		rate = rospy.Rate(10) # 10hz
		#tell the action client that we want to spin a thread by default
		self.Pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Publish Nav Goal to ROS topic
		self.count = 0
		while not rospy.is_shutdown():

			rospy.Subscriber('/outdoor_waypoint_nav/odometry/filtered',Odometry, self.robotPoseSubscriber)
			rospy.Subscriber('/rover_state_topic',RoverStateMsg, self.stateSubscriber)
			rospy.Subscriber('/bearing_to_ball',String, self.ballYawSubscriber)
			rospy.Subscriber('/move_base/result',MoveBaseActionResult, self.moveSubscriber)
			print(str(self.state.state))
			if(self.state.state==self.state.REACH_IMAGE):
				if(self.msg =="-"):
				   print("ball is not found")
				   twist_empty=Twist()
				   self.Pub.publish(twist_empty)
		

				else:
				   bear = abs(float(self.msg))
				   self.bearingToball= float(self.msg)*pi /180
				   print(self.msg)
				   if bear> 5:
						self.rotate_to_ball_2()  

				   elif bear <= 5:
						self.twist.angular.z=0
						self.Pub.publish(self.twist)
						if(self.count<3):
							self.go_forward()
						else:
							while not rospy.is_shutdown():
								print("Succesful")
		

				

			#rospy.spin()    
	def stateSubscriber(self,stateMsg):
		self.state=stateMsg

	def ballYawSubscriber(self,yawMsg):
		self.msg=yawMsg.data 
		#print(self.msg)          
	
	def rotate_to_ball_2(self):
		if(self.bearingToball>0):
			self.twist.angular.z=0.1
		if(self.bearingToball<0):
			self.twist.angular.z=-0.1
                      

		self.Pub.publish(self.twist)
  
	   

	def go_forward(self):

		goal=MoveBaseGoal()
		goal.target_pose.header.frame_id = "/base_link"
		dist=1
		goal.target_pose.pose.position.x = dist
		goal.target_pose.pose.position.y =0
		goal.target_pose.pose.position.z = 0

		q = tf.transformations.quaternion_from_euler(0,0,0)
		goal.target_pose.pose.orientation.x = q[0]
		goal.target_pose.pose.orientation.y = q[1]
		goal.target_pose.pose.orientation.z = q[2]
		goal.target_pose.pose.orientation.w = q[3] 

		self.client.send_goal(goal)
		wait = self.client.wait_for_result()
		self.count += 1
	

if __name__ == '__main__':
	try:
		GoForwardAvoid()
	except rospy.ROSInterruptException:
		rospy.loginfo("Exception thrown")
