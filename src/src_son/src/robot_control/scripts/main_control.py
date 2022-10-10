#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time
import math
import numpy as np
import cv2
import os
import sys
from line_track_camera import *


class Main_Control:
    def __init__(self, senaryono, senaryotype):
        
        self.all_parameters = rospy.get_param('/robot_params')
        self.all_qr = rospy.get_param('/robot_params/qrs')
        self.qr_data = []  # create empty qr code list
        self.bossenaryolar = rospy.get_param('/robot_params/bosenaryolar')
        self.yuklusenaryolar = rospy.get_param('/robot_params/yuklusenaryolar')
        self.yuk_bos_nokta = rospy.get_param('/robot_params/yukbosnokta')
        self.kose_nokta = rospy.get_param('/robot_params/kosenokta')
        self.engel_nokta = rospy.get_param('/robot_params/engelnokta')
        self.basbitis_nokta = rospy.get_param('/robot_params/basbitisnokta')

        self.subs_encoder = rospy.Subscriber(
            "encoder_data", String, self.callback_encoder)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.subs_qr = rospy.Subscriber("qr_topic", String, self.callback_qr)
        self.rate = rospy.Rate(1)
        self.list_parameters = []
        self.rate = rospy.Rate(1)  # 1hz rate for the loop
        if senaryotype == "bos":

            self.run_race(
                self.bossenaryolar[senaryono], senaryono, senaryotype)

        elif senaryotype == "yuklu":
            self.run_race(
                self.yuklusenaryolar[senaryono], senaryono, senaryotype)

        else:
            rospy.logdebug(" enter correct senaryo type")
            rospy.loginfo(" example: yuklu, bos")

    def callback_qr(self, data):
        qr = data.data
        self.qr_data = qr.split(";")  # qr_data is a list of [qr_id,qr_x,qr_y]

    def callback_encoder(self, encoder):
        pass

    def write_param(self):
        for i in self.all_parameters:
            self.list_parameters.append(i)
            rospy.set_param(i, self.all_parameters[i])
        print(self.list_parameters)
        print("*"*20)
        print(self.all_parameters)
        print("+"*20)
        print(type(self.all_parameters))
        rospy.loginfo("Parameters are written")

    def run_race(self, senaryo, senaryono, senaryotype):
        # print(senaryo)
        # print(type(senaryo))

        kararlar = senaryo[2]
        hedefler = senaryo[3]
        yuknoktaları = []

        if len(senaryo) > 4:
            yuknoktaları = senaryo[4]

        # print(kararlar)
        # print(hedefler)
        # print(type(hedefler))
        zeroandzero = 0
        zeroorzero = 0
        whatelse = 0

        for item in zip(kararlar["karar"].split("-"), hedefler["hedef"].split("-")):
            print(item)
            # hqr is a list of [qr_id,qr_x,qr_y]
            hqr = self.all_qr[item[1]].split()
            # kqr is a list of [qr_id,qr_x,qr_y]
            kqr = self.all_qr[item[0]].split()

            h_x = hqr[0]
            h_y = hqr[1]
            k_x = kqr[0]
            k_y = kqr[1]
            diff_x = int(h_x) - int(k_x)
            diff_y = int(h_y)-int(k_y)
            print(f" {item[0]}-{item[1]} : {diff_x}  {diff_y}")

            if diff_x == 0 and diff_y == 0:
                zeroandzero += 1
                # start line tracking from kqr to hqr first time
                if zeroandzero == 1:
                    print(f"seroand zero :{zeroandzero}")
                    print(f"start line tracking from {kqr} to {hqr}")
                    while True:

                        self.line_track()
                        if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                            print(f"{kqr} to {hqr} is reached")
                        break

                else:
                    print(f"stop robot {kqr} to {hqr}")

                    self.move_stop()

            elif diff_x == 0 or diff_y == 0:
                zeroandzero += 1
                while True:
                    print(f"go straight from {kqr} to {hqr}")
                    self.line_track()
                    if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                        print(f"{kqr} to {hqr} is reached")
                        break

            else:
                whatelse += 1
                if senaryotype == "yuklu":
                    # print(senaryo == "senaryo1")
                    # print(type(senaryo))
                    # print(senaryo)
                    if senaryono == "senaryo1":

                        while whatelse <= 4:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()  # sleep for 1 second
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break
                            break

                        while 4 < whatelse <= 10:
                            while True:
                                print(
                                    f"turn left ({whatelse}) from {kqr} to {hqr}")
                                self.move_turnLeft(kqr, hqr)
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break
                            break

                    elif senaryono == "senaryo2":

                        while whatelse <= 6:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break

                            break

                        while 6 < whatelse <= 10:
                            while True:
                                print(
                                    f"turn left ({whatelse}) from {kqr} to {hqr}")
                                self.move_turnLeft(kqr, hqr)
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break
                            break

                    elif senaryono == "senaryo3":

                        while whatelse <= 4:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break
                            break

                        while 4 < whatelse <= 10:
                            while True:
                                print(
                                    f"turn left ({whatelse}) from {kqr} to {hqr}")
                                self.move_turnLeft(kqr, hqr)
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break

                            break

                    elif senaryono == "senaryo4":

                        while whatelse <= 6:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break

                            break

                        while 6 < whatelse <= 10:
                            while True:
                                print(
                                    f"turn left ({whatelse}) from {kqr} to {hqr}")
                                self.move_turnLeft(kqr, hqr)
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break

                            break
                    else:
                        print("enter correct senaryo number")
                        print("example: senaryo1, senaryo2, senaryo3, senaryo4")
                        break

                else:
                    if senaryono == "senaryo1":

                        while whatelse <= 4:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break
                            break
                    elif senaryono == "senaryo2":

                        while whatelse <= 4:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break

                            break
                    elif senaryono == "senaryo3":

                        while whatelse <= 4:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break

                            break
                    elif senaryono == "senaryo4":

                        while whatelse <= 4:
                            while True:
                                self.move_turnRight(kqr, hqr)
                                print(
                                    f"turn right ({whatelse}) from {kqr} to {hqr}")
                                self.rate.sleep()
                                if self.qr_data[0] == hqr[0] and self.qr_data[1] == hqr[1] and self.qr_data[2] == hqr[2]:
                                    print(f"{hqr} reached")
                                    break

                            break
                    else:
                        print("enter correct senaryo number")
                        print("example: senaryo1, senaryo2, senaryo3, senaryo4")
                        break

            time.sleep(1)

        print(f"zeroandzero: {zeroandzero}")
        print(f"zeroorzero: {zeroorzero}")
        print(f"whatelse: {whatelse}")

    def line_track(self):
        line_track = LineTrackCamera()
        main(line_track)

    def move_straight(self):
        pass

    def move_turn(self):
        pass

    def pass_avodiance(self):
        pass

    def move_turnRight(self, kqr, hqr):
        cmd_msg = Twist()
        cmd_msg.linear.x = 1
        cmd_msg.angular.z = -0.025
        self.cmd_vel_pub.publish(cmd_msg)
        print("turn right")

    def move_turnLeft(self, kqr, hqr):

        cmd_msg = Twist()
        cmd_msg.linear.x = 1
        cmd_msg.angular.z = 0.025
        self.cmd_vel_pub.publish(cmd_msg)
        print("turn left")

    def move_stop(self, kqr, hqr):

        cmd_msg = Twist()
        cmd_msg.linear.x = 0
        cmd_msg.angular.z = 0
        self.cmd_vel_pub.publish(cmd_msg)
        print("stop")


if __name__ == "__main__":
    try:
        rospy.init_node("main_control")
        # get input frrom terminal and run the program
        args = rospy.myargv(argv=sys.argv)  # get input from terminal
        if len(args) > 2:
            senaryotype = args[1]
            senaryono = args[2]
            print(senaryotype)
            print(senaryono)
            main_control = Main_Control(senaryono, senaryotype)
            # main_control.write_param()
        else:
            print("no arguments...enter senaryotype,senaryo")
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
        pass
    finally:
        pass
