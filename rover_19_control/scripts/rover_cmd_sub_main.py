#!/usr/bin/env python
# 2018 URC and 2018 ERC subs joy mobile_base,  pubs to serial node
# 1-3 are the left side of the mobile_base, 2-4 are the right side of the mobile_base.
# "S + motor_1 + motor_2 + motor_3  + motor_4 + F"
# İf you dont use your joy, it will send 0000 to serial node
# ITU mobile_base Team
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist


wayLeft="0"
speed1String="000"
wayRight="000"
speed2String="000"
twist =Twist();
pub=rospy.Publisher("/mobile_base_serial_topic", String, queue_size=50)
def callbackcmd(data):  
	twist.linear.x = data.linear.x 
	twist.angular.z = -data.angular.z
	
def callbacknav(data):
	global wayLeft, speed1String, wayRight,speed2String
	twist.linear.x = data.linear.x *6
	twist.angular.z = -data.angular.z*10
	pub.publish("S" + str(wayLeft) + str(speed1String) + str(wayLeft) + str(speed1String) + str(wayRight) + str(speed2String) + str(wayRight) + str(speed2String)+ "F")
	
def main():
	global wayLeft, speed1String, wayRight,speed2String
	rospy.init_node('mobile_base_cmd_sub_serial')
	rospy.Subscriber("/mobile_base_joy/cmd_vel", Twist, callbackcmd)
	rospy.Subscriber("/mobile_base_navigation/cmd_vel", Twist, callbacknav)
	b=0.71
	leftWheel =0
	rightWheel=0
	cmdRadius =0
	
	while not rospy.is_shutdown():
		if(twist.angular.z != 0):

			cmdRadius = twist.linear.x/(twist.angular.z+0.0001)
			leftWheel  = (twist.angular.z * (cmdRadius +b/2))*166
			rightWheel =  (twist.angular.z * (cmdRadius - b/2))*166
		else:
			leftWheel = (twist.linear.x)*166
			rightWheel = (twist.linear.x)*166
			
	   

		wayLeft = 0
		wayRight = 0
		
		if(leftWheel>= 0):
			wayLeft=0
		else:
			wayLeft=1
		
		if(rightWheel>= 0):
			wayRight=1
		else:
			wayRight=0

		speed1String = str(abs(int(leftWheel)))
		speed2String = str(abs(int(rightWheel)))
		
		if abs(leftWheel) < 10:
			speed1String = "00" + str(abs(int(leftWheel)))
			
		elif abs(leftWheel) < 100:
			speed1String = "0" + str(abs(int(leftWheel)))
			

		if abs(rightWheel) < 10:
			speed2String = "00" + str(abs(int(rightWheel)))
			
		elif abs(rightWheel) < 100:
			speed2String = "0" + str(abs(int(rightWheel)))

		
		print("S" + str(wayLeft) + str(speed1String) + "," + str(wayLeft) + str(speed1String) + "," + str(wayRight) + str(speed2String) + ","+ str(wayRight) + str(speed2String)+ "F")
		pub.publish("S" + str(wayLeft) + str(speed1String) + str(wayLeft) + str(speed1String) + str(wayRight) + str(speed2String) + str(wayRight) + str(speed2String)+ "F")
		
	rospy.spin()

if __name__ == '__main__':
	main()
