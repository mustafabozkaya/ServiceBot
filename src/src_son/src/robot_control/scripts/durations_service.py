#!/usr/bin/env python3
import rospy
import time
from robot_control.srv import Durations,DurationsResponse
from geometry_msgs.msg import Twist

class DurationService():
    

    def __init__(self,*args) -> None:
        if len(args)>=1:
            move_topic=args[0]
            rospy.init_node('service_move_custom_server') 
            self.my_service = rospy.Service(f'/{move_topic}', Durations , self.my_callback) # create the Service called move_bb8_in_circle with the defined callback
            self.my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            self.move_circle = Twist()
            self.rate = rospy.Rate(1)
            rospy.loginfo(f"Service /{move_topic} Ready")
            rospy.spin() # mantain the service open.
        else:
             rospy.init_node("Duration_service_node")
             self.server=rospy.Service("duration_service",Durations,self.request_handle)
             self.rate = rospy.Rate(1)
             rospy.loginfo("Service duration_service Ready")
             rospy.spin() # mantain the service open.


    def request_handle(self,request):
        rospy.loginfo("duration service called")
        self.speed=0.5
        self.response=DurationsResponse()
        duration=request.target/self.speed

        self.response.duration=duration
        return self.response


    def my_callback(self,request):
        rospy.loginfo(f"The Service move_otonom has been called")
        self.cmd_vel.linear.x = 0.2
        self.cmd_vel.angular.z = 0.2
        i = 0
        while i <= request.duration: 
            self.my_pub.publish(self.cmd_vel)
            self.rate.sleep()
            i=i+1
            
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.my_pub.publish(self.cmd_vel)
        rospy.loginfo("Finished service move_otonom")
        
        response = DurationsResponse()
        response.success = True
        return response # the service Response class

if __name__=="__main__":
    duration=DurationService()

