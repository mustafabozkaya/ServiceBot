#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import os
from datetime import datetime
from std_msgs.msg import String
import rospy
import datetime 


if __name__ == '__main__':
    rospy.init_node('qr_node')
    pub = rospy.Publisher('qr_topic', String, queue_size=10)
    try:
            port = rospy.get_param(
                '/robot_params/qr_port', default="/dev/ttyUSB0")
            print("Port listening ", port)
            # give permission to port
            os.system("sudo chmod 666 " + port)
            # open port and read data from it
            serialconnect = serial.Serial(port, bytesize=8, baudrate=115200)

            while not rospy.is_shutdown():
                try:
                    if serialconnect.inWaiting() > 0:  # if data is available ,inWaiting() returns the number of bytes available to read in the input buffer
                        # read data from serial port and decode it to utf-8
                        recevied_data = serialconnect.read().decode("utf-8")
                        print(recevied_data)
                        print(type(recevied_data))
                        qr_data = ""  # create empty string
                        for i in recevied_data:
                            if i != '\r':  # if i is not a new line character
                                qr_data += i
                        if qr_data != "":
                            pub.publish(qr_data)  # publish data
                            rospy.loginfo(datetime.now().strftime(
                                "%H:%M:%S") + " " + qr_data)
                            print(datetime.now().strftime(
                                "%H:%M:%S") + " " + qr_data)

                except Exception as e:
                    print(e)
                    print("Error in reading data")
                    break

    except:
            print("Error in connecting to port")
            
    rospy.spin()
    serialconnect.close()
    print("Serial port closed")
    rospy.loginfo("Serial port closed")
    rospy.loginfo("QR node closed")
