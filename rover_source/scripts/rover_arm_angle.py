#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int8
from math import pi
import numpy
from rover_source.msg import *


joint = Joint_msgs()
moveit_commander.roscpp_initialize(sys.argv)

group_name = "arm_rover"
group = moveit_commander.MoveGroupCommander(group_name)

class Angle(object):
    def __init__(self):
        self.gripper = Int8()
        self.main()
    def gripper_callback(self,data):

        self.gripper = data.data

    def main(self):
        rospy.Subscriber("/gripper_command_19", Int8, self.gripper_callback)
        pub1 = rospy.Publisher("/arm19_serial",String,queue_size=5)
        pub2 = rospy.Publisher("/arm_19_ui",String, queue_size=5)
        pub3 = rospy.Publisher("/arm19_sar", Joint_msgs, queue_size=5)
        rospy.init_node('rover_joint_angles',anonymous=True)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            current_joint_values = group.get_current_joint_values()

            joint1 = (current_joint_values[0]*180)/pi
            joint2 = (current_joint_values[1]*180)/pi
            joint3 = (current_joint_values[2]*180)/pi
            joint4 = (current_joint_values[3]*180)/pi
            joint5 = (current_joint_values[4]*180)/pi
            joint6 = (current_joint_values[5]*180)/pi


            joint1_str = self.floattostring(joint1)
            joint2_str = self.floattostring(joint2)
            joint3_str = self.floattostring(joint3)
            joint4_str = self.floattostring(joint4)
            joint5_str = self.floattostring(joint5)
            joint6_str = self.floattostring(joint6)
            gripper_str= str(self.gripper)

            print "S"+joint1_str+" "+joint2_str+" "+joint3_str+" "+joint4_str+" "+joint5_str+" "+joint6_str+" "+gripper_str+"C"+"F"
            pub1.publish("S"+joint1_str+joint2_str+joint3_str+joint4_str+joint5_str+joint6_str+gripper_str+"C"+"F")
            pub2.publish(joint1_str+" "+joint2_str+" "+joint3_str+" "+joint4_str+" "+joint5_str+" "+joint6_str+" "+gripper_str)
            rate.sleep()
    def floattostring(self,joint):
        if joint<0 :
            value = int((-1)*joint)
            if value<10:
                string = "000"+str(value)
            elif value< 100 and value > 9:
               string = "00"+str(value)
            else:
                string = "0"+str(value)

        else :
            value= int(joint)
            if value<10:
                string= "100"+str(value)
            elif value < 100 and value > 9:
                string = "10"+str(value)
            else:
                string = "1"+str(value)
        return string

if __name__ == '__main__':
    try:
        Angle()
    except rospy.ROSInterruptException:
        pass
