#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from robot_control.msg import BatteryStat


def battery_callback(msg):
    rospy.loginfo("Robot şarjı: %s"%msg.batarya)

def batterysubscriber():
    rospy.init_node("battery_node")
    rospy.Subscriber("battery_t",BatteryStat,battery_callback)
    rospy.spin()

if __name__=="__main__":
    batterysubscriber()