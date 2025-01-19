#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32MultiArray
import math

ser = serial.Serial("/dev/ttyAMA0", 115200)

def callback(data):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    start = 181 if data.data[0] < 0 else 182
    angles = int(math.fabs(data.data[0]))

    ser.write(start.to_bytes(1, 'little'))
    ser.write(angles.to_bytes(1, 'little'))

def listener():
    rospy.init_node('rpi')

    rospy.Subscriber("pcout", Int32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
