#!/usr/bin/env python3
import re
import rospy
import time
from tatus.srv import Lift, LiftResponse, LiftRequest
from tatus.srv import Buzzer, BuzzerResponse, BuzzerRequest
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Lift_Buzzer():

    def __init__(self) -> None:
        super().__init__()
        rospy.init_node('service_move_custom_server')

        rospy.Service(
            'lift_service', Lift, self.requestlift)
        rospy.Service(
            'buzzer_service', Buzzer, self.request_buzzer)
        self.liftpub = rospy.Publisher('/lift', String, queue_size=10)
        self.buzzerpub = rospy.Publisher('/buzzer', String, queue_size=10)

        self.rate = rospy.Rate(1)
        rospy.loginfo(f"Service  lift buzzer Ready")
        rospy.spin()  # mantain the service open.

    def requestlift(self, req):
        lift_msg = String()
        self.response = LiftResponse()
        rospy.loginfo("lift  service called")

        qr = req.qrcode
        lift_up = req.lift_up
        payload = req.payload

        if lift_up == "up":
            lift_msg = "up"
            buzzer_msg = "on"
            i = 1
            while not rospy.is_shutdown() and i <= 15:
                self.liftpub.publish(lift_msg)
                self.buzzerpub.publish(buzzer_msg)
                rospy.loginfo("lift is up")

                self.rate.sleep()
                i = i+1
            buzzer_msg = "stop"
            lift_msg = "stop"
            self.liftpub.publish(lift_msg)
            self.buzzerpub.publish(buzzer_msg)
            self.response.success = True
        elif lift_up == "down":
            i = 1
            lift_msg = "down"
            buzzer_msg = "on"
            while not rospy.is_shutdown() and i <= 10:
                self.liftpub.publish(lift_msg)
                self.buzzerpub.publish(buzzer_msg)
                self.rate.sleep()
                i = i+1
            lift_msg = "stop"
            buzzer_msg = "stop"
            self.buzzerpub.publish(buzzer_msg)
            self.liftpub.publish(lift_msg)
            self.response.success = True
        else:
            lift_msg = "stop"
            buzzer_msg = "stop"
            self.buzzerpub.publish(buzzer_msg)
            self.liftpub.publish(lift_msg)

    def request_buzzer(self, req):
        buzer_msg = String()
        rospy.loginfo("buzzer service called")
        self.response = BuzzerResponse()
        rospy.loginfo(f"request buzzer {req.buzzer_on}")

        try:

            if req.buzzer_on:
                buzer_msg = True
                # publish one message to the topic
                self.buzzerpub.publish(buzer_msg)

                rospy.loginfo("buzzer on")
                self.response.success = True
            elif not req.buzzer_on:
                buzer_msg = False
                self.buzzerpub.publish(buzer_msg)
                rospy.loginfo("buzer on ")
                self.response.success = True

        except Exception as e:
            rospy.loginfo(f"error {e}", e)
            self.response.success = False

        return self.response


if __name__ == "__main__":
    duration = Lift_Buzzer()
