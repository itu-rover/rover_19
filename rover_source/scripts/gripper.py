#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8

gripper_command = Int8()
def callback(data):

    global gripper_command
    gripper_command = data.data

def baturay(gripper_command):
    if str(gripper_command) == "data: 0" :
        gripper_command = 2
    else :
        gripper_command = gripper_command
    return gripper_command
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/gripper_command19", Int8, callback)
    pub = rospy.Publisher("/gripper_command_19", Int8, queue_size=10)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        a = baturay(gripper_command)
        print(a)
        pub.publish(a)
        rate.sleep()

if __name__ == '__main__':
    listener()
