#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
convert to ros img to opencv img and show it
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class RobotKamera():
    def __init__(self):
        rospy.init_node("kamera")
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.cameraCallback)
        self.bridge = CvBridge()
        self.width = 480
        self.height = 360
        self.foto = np.uint8([[[0, 0, 0]]])
        self.CreateTrackBar_Init()
        rospy.spin()

    def cameraCallback(self, mesaj):
        self.foto = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        self.foto = cv2.resize(self.foto, (self.width, self.height))

        self.get_hsv_img()

    def get_hsv_img(self):

        frame = cv2.resize(self.foto, (640, 480))
        crop_witdth = int(frame.shape[1] * 0.25)
        crop_height = int(frame.shape[0] * 0.40)
        frame = frame[crop_height:, crop_witdth:]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        (low_h, low_s, low_v, high_h, high_s, high_v) = self.get_trackbar_pos()

        lower_color = np.array([low_h, low_s, low_v])
        upper_color = np.array([high_h, high_s, high_v])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        # erode the mask to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        # dilate the mask to fill in gaps
        mask = cv2.dilate(mask, None, iterations=2)
        # blur the mask to make the edges clearer
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball

        result = cv2.bitwise_and(frame, frame, mask=mask)

        # cv2.imshow("ORİGİNAL İMG", frame)
        cv2.imshow("MASK", mask)
        # cv2.imshow("RESULT", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            print("Programdan çıkılıyor...")

    def CreateTrackBar_Init(self):
        cv2.namedWindow("Trackbars")  # create a window for trackbars
        cv2.createTrackbar("LowerH", "Trackbars", 0, 179,
                           self.nothing)  # hue 0-179
        cv2.createTrackbar("LowerS", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("LowerV", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("UpperH", "Trackbars", 169, 179, self.nothing)
        cv2.createTrackbar("UpperS", "Trackbars", 204, 255, self.nothing)
        cv2.createTrackbar("UpperV", "Trackbars", 204, 255, self.nothing)

    def get_trackbar_pos(self):
        low_h = cv2.getTrackbarPos("LowerH", "Trackbars")
        low_s = cv2.getTrackbarPos("LowerS", "Trackbars")
        low_v = cv2.getTrackbarPos("LowerV", "Trackbars")
        high_h = cv2.getTrackbarPos("UpperH", "Trackbars")
        high_s = cv2.getTrackbarPos("UpperS", "Trackbars")
        high_v = cv2.getTrackbarPos("UpperV", "Trackbars")
        return low_h, low_s, low_v, high_h, high_s, high_v

    def nothing(self, x):
        pass


if __name__ == '__main__':
    nesne = RobotKamera()
