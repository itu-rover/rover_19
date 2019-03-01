#!/usr/bin/env python

# 5 situation for controlling sledge ->  0 = down, 1 = slowly down, 2 = stop, 3 = slowly up, 4 = up
# 4 situation for Auger tray ->  0 = (red => downward), 1 = (blue => downward), 2 = (yellow => downward), 3 = (white => downward)
# 2 situation for Auger control -> 1 = start, 0 = stop
# 7 situation for Probe control -> 1 = red probe, 2 = blue probe, 3 = yellow probe, 4 = white probe, 5 = ec, 6 = ph , 7 = chemical
# 2 situation for peristaltic pump -> 1 = start, 0 = stop

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8

def callback(data):

    science_string = data.data
    sledge_position_ui = science_string[1:2]
    auger_tray_ui = science_string[2:3]
    auger_command_ui = science_string[3:4]
    probe_tray_ui = science_string[4:5]
    peristaltic_pump_ui = science_string[5:6]

    sledge_position = sledge_control(int(sledge_position_ui))
    auger_tray = auger_tray_control(auger_tray_ui)
    auger_command = auger_command_control(auger_command_ui)
    probe_tray = probe_tray_control(probe_tray_ui)
    peristaltic_pump = peristaltic_pump_control(peristaltic_pump_ui)

    sledge_position_string = inttostring(sledge_position)
    auger_tray_string = str(auger_tray)
    auger_command_string =  str(auger_command)
    probe_tray_string = str(probe_tray)
    peristaltic_pump_string = str(peristaltic_pump)
    science_serial_msg = String()
    science_serial_msg.data = "S"+sledge_position_string+auger_tray_string+auger_command_string+probe_tray_string+peristaltic_pump_string+"C"+"F"
    print("Science data = %s  " %science_serial_msg.data)
    pub.publish(science_serial_msg.data)

def sledge_control(value):
    sledge_elk = Int8()
    if value == 0 :
        sledge_elk = -500
    elif value == 1 :
        sledge_elk = -100
    elif value == 2 :
        sledge_elk =  0
    elif value == 3 :
        sledge_elk = 100
    elif value == 4 :
        sledge_elk = 500
    return sledge_elk

def auger_tray_control(value):
    auger_tray_elk = 0
    auger_tray_elk = value

    return auger_tray_elk

def auger_command_control(value):
    auger_command_elk = 0
    auger_command_elk = value

    return auger_command_elk

def probe_tray_control(value):
    probe_tray_control_elk = 0
    probe_tray_control_elk = value

    return probe_tray_control_elk

def peristaltic_pump_control(value):
    peristaltic_pump_elk = 0
    peristaltic_pump_elk = value

    return peristaltic_pump_elk

def inttostring(elk):
    if elk < 0 :
        value =  int((-1) * elk)
        if value<10:
            string = "000"+str(value)
        elif value< 100 and value > 9:
           string = "00"+str(value)
        else:
            string = "0"+str(value)

    else :
        value= int(elk)
        if value<10:
            string= "100"+str(value)
        elif value < 100 and value > 9:
            string = "10"+str(value)
        else:
            string = "1"+str(value)
    return string

def main():
    global pub
    pub = rospy.Publisher("/science_serial",String,queue_size=50)
    rospy.Subscriber("/science_ui", String, callback)
    rospy.init_node('rover_science',anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main()
