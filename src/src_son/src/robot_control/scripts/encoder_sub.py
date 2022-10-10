#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32


class EncoderSub:
    def __init__(self):
        self.encoder_sub = rospy.Subscriber('encoder', Int32, self.encoder_callback)
       
        self.encoder_count = 0
        self.prev_encoder_count = 0

    def encoder_callback(self, data):
        self.prev_encoder_count = self.encoder_count
        self.encoder_count = data.data
        