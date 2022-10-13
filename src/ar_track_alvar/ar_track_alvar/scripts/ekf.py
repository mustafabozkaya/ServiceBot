#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from time import time
import rospy
from geometry_msgs.msg import Twist

class Ekf():
    def __init__(self):
        rospy.init_node("EKF")
        self.rate =rospy.Rate(5)
        self.cmd_ar_vel = 0
        self.cmd_line_vel = 0
        self.cmd_avoid_vel = 0
        self.cmd_vel = Twist() 

        
        rospy.Subscriber("ar_vel", Twist, 
        self.ar_vel)
        rospy.Subscriber("line_vel", Twist, 
        self.line_vel)
        rospy.Subscriber("avoid_vel", Twist, 
        self.avoid_vel)

        self.pub = rospy.Publisher(
            "cmd_vel", Twist, queue_size=10)
    
    def ar_vel(self,msg):
        self.cmd_ar_vel = msg
        # print("v2")
        # print(self.cmd_ar_vel)
    
    def line_vel(self,msg):
        self.cmd_line_vel = msg
        # print("v3")
        # print(self.cmd_line_vel)
        # print("----------------")
    
    def avoid_vel(self,msg):
        self.cmd_avoid_vel = msg
    
    def toplama(self):

        return (self.cmd_ar_vel,self.cmd_line_vel)
    def type_1(self,ar,line):
        if ar.linear.x !=0 and line.linear.x !=0:
            print("iki hedef var")
            line_track = 0.5
            ar_tag = 0.5
        elif  ar.linear.x ==0 and line.linear.x !=0:
            print("Line_Track")
            line_track = 1
            ar_tag = 0
        elif ar.linear.x !=0 and line.linear.x ==0:
            print("Ar_tag")
            line_track = 0
            ar_tag = 1
        else:
            print("Hedef Yok")
            line_track = 0
            ar_tag = 0
        return ar_tag,line_track
    def type_2(self,ar,line):
        if line.linear.x !=0:
            print("Line Track")
            line_track = 1
            ar_tag = 0
        elif ar.linear.x !=0 and line.linear.x ==0:
            print("Ar_tag")
            line_track = 0
            ar_tag = 1
        else:
            print("Hedef Yok")
            line_track = 0
            ar_tag = 0
        return ar_tag,line_track

    def type_3(self,ar,line,avoid):
        if avoid.angular.z != 0 or avoid.linear.x == 0:
            print("Kaçınma")
            self.pub.publish(avoid)
            
        else:
            if line.linear.x !=0:
                print("Line Track")
                line_track = 1
                ar_tag = 0
            elif ar.linear.x !=0 and line.linear.x ==0:
                print("Ar_tag")
                line_track = 0
                ar_tag = 1
            else:
                print("Hedef Yok")
                line_track = 0
                ar_tag = 0

            if (ar_tag + line_track)<1:
                rospy.loginfo("Hatalı değer")
                self.cmd_vel.linear.x = 0
                self.cmd_vel.linear.y = 0
                self.cmd_vel.linear.z = 0 
                self.cmd_vel.angular.x = 0
                self.cmd_vel.angular.y = 0
                self.cmd_vel.angular.z = 0
                self.pub.publish(self.cmd_vel)
                pass
            else:
                self.cmd_vel.linear.x = ((ar.linear.x * ar_tag)+
                    line.linear.x*line_track)
                if ar.linear.x ==0:
                    self.cmd_vel.linear.x = 0.0
                # elif line.linear.x !=0 and ar.linear.x !=0:
                #     self.cmd_vel.linear.x = 0.15
                else:    
                    self.cmd_vel.linear.x = 0.10
                self.cmd_vel.linear.y = ((ar.linear.y * ar_tag)+
                    line.linear.y*line_track)
                self.cmd_vel.linear.z = ((ar.linear.z * ar_tag)+
                    line.linear.z*line_track)
                self.cmd_vel.angular.x = ((ar.angular.x * ar_tag)+
                    line.angular.x *line_track)
                self.cmd_vel.angular.y = ((ar.angular.y * ar_tag)+
                    line.angular.y*line_track)
                self.cmd_vel.angular.z = ((ar.angular.z * ar_tag)+
                    line.angular.z*line_track)
                self.pub.publish(self.cmd_vel)




    def average(self):
        #ar ar_tag cmd verisidir line line cmd verisidir
        ar = self.cmd_ar_vel
        line = self.cmd_line_vel
        avoid = self.cmd_avoid_vel
        if ar != 0 and line !=0:
            # self.type_3(ar,line,avoid)
            
            
            ar_tag,line_track=self.type_2(ar,line)
            if (ar_tag + line_track)<1:
                rospy.loginfo("Hatalı değer")
                self.cmd_vel.linear.x = 0
                self.cmd_vel.linear.y = 0
                self.cmd_vel.linear.z = 0 
                self.cmd_vel.angular.x = 0
                self.cmd_vel.angular.y = 0
                self.cmd_vel.angular.z = 0
                self.pub.publish(self.cmd_vel)
                pass
            else:
                self.cmd_vel.linear.x = ((ar.linear.x * ar_tag)+
                    line.linear.x*line_track)
                if ar.linear.x ==0:
                    self.cmd_vel.linear.x = 0.0
                else:
                    
                    self.cmd_vel.linear.x = 0.15
                self.cmd_vel.linear.y = ((ar.linear.y * ar_tag)+
                    line.linear.y*line_track)
                self.cmd_vel.linear.z = ((ar.linear.z * ar_tag)+
                    line.linear.z*line_track)
                self.cmd_vel.angular.x = ((ar.angular.x * ar_tag)+
                    line.angular.x *line_track)
                self.cmd_vel.angular.y = ((ar.angular.y * ar_tag)+
                    line.angular.y*line_track)
                self.cmd_vel.angular.z = ((ar.angular.z * ar_tag)+
                    line.angular.z*line_track)
                self.pub.publish(self.cmd_vel)
                
        else:
            self.cmd_vel.linear.x = 0
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0 
            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = 0
            self.pub.publish(self.cmd_vel)
        return self.cmd_vel

if __name__ == "__main__":
    ekf=Ekf()
    while(not rospy.is_shutdown()):	
        
        v2 ,v3 = ekf.toplama()
        if v2 != 0 and v3 !=0:
            
            ekf.average()
            # print(v2.linear.x)
            # print(v3.linear.x)
            # differ = abs(v2.angular.z-v3.angular.z)
            # print(differ)
            # print(ekf.average()
        ekf.rate.sleep()
            
 


