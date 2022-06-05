#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8

import serial

ser = None

def callback(data):

    packet = bytearray()
    packet.append(data.data)
    packet.append(0x00) 
    ser.write(packet)

    rospy.loginfo("sent to device " + hex(data.data))
    

if __name__ == '__main__':
    rospy.init_node('serial_node')
    ser = serial.Serial('/dev/rfcomm0')

    rospy.Subscriber("send_to_device", UInt8, callback)
    pub = rospy.Publisher('received_from_device', UInt8, queue_size=10)

    while not rospy.is_shutdown():
        received_message_str_hex = ser.readline().hex()[:2] #only taking the first byte
        received_message_int = int(received_message_str_hex,16)
        pub.publish(received_message_int)
        rospy.loginfo("received from device:" + hex(received_message_int))


     
