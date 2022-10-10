#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
get real camera image and publish it to /camera/rgb/image_raw
"""

import cv2
import numpy as np
#mavi = np.uint8([[[255,0,0]]],dt=np.uint8)
#hsv = cv2.cvtColor(mavi,cv2.COLOR_BGR2HSV)
# print(hsv)


def CreateTrackBar_Init():
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("Lower - H", "Trackbars", 0, 179, nothing)  # hue 0-179
    cv2.createTrackbar("Lower - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("Lower - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("Upper - H", "Trackbars", 169, 179, nothing)
    cv2.createTrackbar("Upper - S", "Trackbars", 204, 255, nothing)
    cv2.createTrackbar("Upper - V", "Trackbars", 204, 255, nothing)


def nothing(self, x):
    pass


CreateTrackBar_Init()
cap = cv2.VideoCapture(0)
while True:

    _, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    Lower_H_Value = cv2.getTrackbarPos("Lower - H", "Trackbars")
    Lower_S_Value = cv2.getTrackbarPos("Lower - S", "Trackbars")
    Lower_V_Value = cv2.getTrackbarPos("Lower - V", "Trackbars")
    Upper_H_Value = cv2.getTrackbarPos("Upper - H", "Trackbars")
    Upper_S_Value = cv2.getTrackbarPos("Upper - S", "Trackbars")
    Upper_V_Value = cv2.getTrackbarPos("Upper - V", "Trackbars")

    lower = np.array([Lower_H_Value, Lower_S_Value, Lower_V_Value])
    upper = np.array([Upper_H_Value, Upper_S_Value, Upper_V_Value])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("ORİGİNAL İMG", frame)
    cv2.imshow("MASK", mask)
    cv2.imshow("RESULT", result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
