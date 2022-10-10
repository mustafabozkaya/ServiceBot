#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
from tatus.srv import Lift, LiftResponse, LiftRequest
from tatus.srv import Buzzer, BuzzerResponse, BuzzerRequest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def lift_client():
    rospy.init_node('lift_client')
    rospy.wait_for_service('/lift_service')
    lift_client = rospy.ServiceProxy('/lift_service', Lift)
    request = LiftRequest()
    request.lift_up = True
    request.payload = True
    response = lift_client(request)
    rospy.loginfo("response result :"+str(response))


if __name__ == '__main__':
    lift_client()
