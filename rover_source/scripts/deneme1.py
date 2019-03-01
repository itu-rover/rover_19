#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

def callback(data):
    asd = String()
    asd.data= "S"+str(int(data.buttons[4]))+"1"+str(2*data.buttons[5])+"11CF"
    pub.publish(asd)

# Intializes everything
def start():
    global pub
    pub = rospy.Publisher('science_ui', String, queue_size=50)
    rospy.Subscriber("joy", Joy, callback)
    rospy.init_node('yavuz')
    rospy.spin()

if __name__ == '__main__':
    start()
