#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import datetime
import rospy

from robot_control.msg import BatteryStat


def msgpublish():
    rospy.init_node("battery_pub",anonymous=True)
    pub = rospy.Publisher("battery_t",BatteryStat,queue_size=10)
    rate = rospy.Rate(1) # 1 hz 
    t0=rospy.Time.now().secs
    start=87.0
    while not rospy.is_shutdown():
        
        state = "%"+str(start)

        t1=rospy.Time.now().secs
        while t1-t0 >=5:
            start-=1
            state="%"+str(start)
            t0=t1

        rospy.loginfo(state)
        pub.publish(state)
        rate.sleep()

msgpublish()