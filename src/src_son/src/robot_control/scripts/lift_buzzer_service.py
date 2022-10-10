#!/usr/bin/env python3
import re
import rospy
import time
from robot_control.srv import Lift,LiftResponse
from robot_control.srv import Buzzer,BuzzerResponse,BuzzerRequest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Lift_Buzzer():

    def __init__(self) -> None:
        
            
            rospy.init_node('service_move_custom_server')
            
            rospy.Service(
                'lift_service', Lift, self.request_lift)
            rospy.Service(
                'buzzer_service', Buzzer, self.request_buzzer)
            self.liftpub = rospy.Publisher('/lift', Twist, queue_size=10)
            self.buzzerpub = rospy.Publisher('/buzzer', Twist, queue_size=10)

            self.rate = rospy.Rate(1)
            rospy.loginfo(f"Service  lift buzzer Ready")
            rospy.spin()  # mantain the service open.
   

    def request_lift(self, req):

        lift_msg=Bool()
        self.response = LiftResponse()
        rospy.loginfo("lift  service called")
        # Wait for the service client /move_bb8_in_circle_custom to be running
        rospy.wait_for_service('/buzzer_service')
        # Create the connection to the service
        buzzer_client = rospy.ServiceProxy('/buzzer_service', Bool)
        # Create an object of type EmptyRequest
        request_buzzer = BuzzerRequest()

        request_buzzer.buzzer_on = True

        
        t0=rospy.time().to_sec()
        try:

            if req.lift_up and (req.payload):
                t1=rospy.time().to_sec()
                i=1
                while i<=15:
                    lift_msg=True
                    self.liftpub.publish(lift_msg)
                    rospy.loginfo("lift is up")
                    rospy.loginfo("buzzer Service Call...")

                    result = buzzer_client(request_buzzer)
                    # Print the result given by the service called
                    rospy.loginfo("response result :"+str(result))

            
                    self.rate.sleep()
                    i=i+1
                request_buzzer.buzzer_on = False
                result = buzzer_client(request_buzzer)
                # Print the result given by the service called
                rospy.loginfo("response result :"+str(result))
                self.response.succes = True

            elif not  req.lift_up and req.payload:
                i = 1
                while i<=10:
                    lift_msg = False
                    result = buzzer_client(request_buzzer)
                    # Print the result given by the service called
                    rospy.loginfo("response result :"+str(result))

                    self.liftpub.publish(lift_msg)
                    rospy.loginfo("lift is down")
                    self.rate.sleep()
                    i=i+1
                    
                request_buzzer.buzzer_on = False
                result = buzzer_client(request_buzzer)
                # Print the result given by the service called
                rospy.loginfo("response result :"+str(result))
                self.response.succes = True
        except Exception as e:
            rospy.loginfo(f"error {e}",e)
            self.response.succes=False

        return self.response

    def request_buzzer(self, req):
         buzer_msg = Bool()
         self.response = BuzzerResponse()
         rospy.loginfo("lift  service called")
         try:

              if req.buzzer_on:
                  buzer_msg = True
                  self.buzzerpub.publish_ones(buzer_msg)
                  rospy.loginfo("buzzer on")
                  self.response.succes = True
              elif not  req.buzzer_on:
                  buzer_msg = False
                  self.buzzerpub.publish_one(buzer_msg)
                  rospy.loginfo("buzer on ")
                  self.response.succes = True

         except Exception as e:
              rospy.loginfo(f"error {e}", e)
              self.response.succes = False

         return self.response


if __name__ == "__main__":
    duration = Lift_Buzzer()
