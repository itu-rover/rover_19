#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 2018 URC  ball searcher node starts when  arrives gps waypoint and could not detect ball 
# bearing_to_ball  topic is a string that gives ball  angle from rover. 
# first, rover is rotating  360 degree . Later it starts to search ball with sending move base goals
# ITU Rover Team
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped,Twist
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rover_state_mach.msg import RoverStateMsg
import tf

class GoForwardAvoid():
    def __init__(self):
        rospy.init_node('ball_search', anonymous=False)

        self.rotate_once=1
        self.dist=1
        self.state=RoverStateMsg()
        self.go_back_counter=0
        self.firstPosX=0
        self.firstPosY=0

        self.state.state=self.state.FIND_IMAGE
        print("waiting move base client...")
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        print("client is on")
        rate = rospy.Rate(10) # 1hz
        #tell the action client that we want to spin a thread by default
        self.Pub = rospy.Publisher('rover_navigation/cmd_vel', Twist, queue_size=10)
        

        while not rospy.is_shutdown():
          
            rospy.Subscriber('/rover_state_topic',RoverStateMsg, self.stateSubscriber)
            #rospy.Subscriber('/move_base/result',MoveBaseActionResult, self.moveSubscriber)
            #print(self.state.state)
            if(self.state.state==self.state.FIND_IMAGE):
                if( self.rotate_once==1):
                    print("searching")
                    self.go_forward()
                    self.rotate(90)
                    self.rotate(90)
                    self.rotate(90)
                    self.rotate(90)
                    self.rotate_once=0

                if(self.state.state==self.state.FIND_IMAGE):
                    self.go_forward()
                    self.go_back_counter +=1
                    if(self.go_back_counter>8):
                        print(str(self.go_back_counter))
                        self.go_back_counter=0
                        self.rotate_once=1
                        self.send_once=1
                        self.do_once=1
                        self.go_to_point(self.firstPosX,self.firstPosY)

            else:
                print("waiting")

        
            rate.sleep()

    def stateSubscriber(self,stateMsg):
        self.state=stateMsg
        if(self.state.state==self.state.REACH_IMAGE):
            if(self.send_once==1):
                print("found image")
                self.twist = Twist()
                self.twist.linear.x=0
                self.twist.angular.z=0
                self.Pub.publish(self.twist)
                self.client.cancel_goal()
                self.client.cancel_all_goals()
                self.send_once=0



    def robotPoseSubscriber(self,poseMsg): #Odometry update recieved from ROS topic, run this function
    
        self.currPosX = poseMsg.pose.pose.position.x
        self.currPosY = poseMsg.pose.pose.position.y
        self.currPosZ = poseMsg.pose.pose.position.z
        self.currOrX = poseMsg.pose.pose.orientation.x
        self.currOrY = poseMsg.pose.pose.orientation.y
        self.currOrZ = poseMsg.pose.pose.orientation.z
        self.currOrW = poseMsg.pose.pose.orientation.w

        quaternion = (
        poseMsg.pose.pose.orientation.x,
        poseMsg.pose.pose.orientation.y,
        poseMsg.pose.pose.orientation.z,
        poseMsg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        #sadece ilk noktayı alır
        if(self.do_once==1):
            self.firstPosX=self.currPosX
            self.firstPosY=self.currPosY
            self.do_once=0

    def go_to_point(self,x,y):

        print("Going To Point...")
        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id = "/base_link"
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y =y
        goal.target_pose.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0,0,0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3] 

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
       
       # print(str(self.dist)

    def rotate(self, angle):     
        print("Rotating...")
        #print(angle)
        goal = MoveBaseGoal()
         
        goal.target_pose.header.frame_id = "/base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x =0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0 

        q = tf.transformations.quaternion_from_euler(0,0,(float(angle)*pi/180))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y =q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3] 
         
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
   
    
if __name__ == '__main__':
    try:
        GoForwardAvoid()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")