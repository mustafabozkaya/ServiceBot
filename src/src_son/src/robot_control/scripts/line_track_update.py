#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
line tracking
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import sys
from robot_control import read_camera
import time
from robot_control import utils_packg as utl
import math
import os


class LineTrack():
    def __init__(self):
        rospy.init_node("track_line")
        self.bridge = CvBridge()
        self.cmd_vel = Twist()

        self.camera_topic = "/camera/color/image_raw"
        rospy.Subscriber("/camera/color/image_raw",
                         Image, self.cameraCallback)

        self.cmd_topic = "/cmd_vel"
        self.pub = rospy.Publisher(
            self.cmd_topic, Twist, queue_size=10)

        self.createTackerbar()

    def createTackerbar(self):
        cv2.namedWindow("HSV trackbar")
        cv2.createTrackbar("low_h", "HSV trackbar", 18, 179, self.nothing)
        cv2.createTrackbar("low_s", "HSV trackbar", 71, 255, self.nothing)
        cv2.createTrackbar("low_v", "HSV trackbar", 0, 255, self.nothing)
        cv2.createTrackbar("high_h", "HSV trackbar", 58, 179, self.nothing)
        cv2.createTrackbar("high_s", "HSV trackbar", 255, 255, self.nothing)
        cv2.createTrackbar("high_v", "HSV trackbar", 255, 255, self.nothing)
        cv2.createTrackbar("point1_x", "HSV trackbar", 204, 640, self.nothing)
        cv2.createTrackbar("point1_y", "HSV trackbar", 100, 480, self.nothing)
        cv2.createTrackbar("point2_x", "HSV trackbar", 474, 640, self.nothing)
        cv2.createTrackbar("point2_y", "HSV trackbar", 100, 480, self.nothing)
        cv2.createTrackbar("point3_x", "HSV trackbar", 65, 640, self.nothing)
        cv2.createTrackbar("point3_y", "HSV trackbar", 480, 480, self.nothing)
        cv2.createTrackbar("point4_x", "HSV trackbar", 640, 640, self.nothing)
        cv2.createTrackbar("point4_y", "HSV trackbar", 480, 480, self.nothing)
        cv2.createTrackbar("camera_w_center", "HSV trackbar",
                           343, 640, self.nothing)

    def nothing(self, x):
        pass

    def getTrackbarPos(self):
        low_h = cv2.getTrackbarPos("low_h", "HSV trackbar")
        low_s = cv2.getTrackbarPos("low_s", "HSV trackbar")
        low_v = cv2.getTrackbarPos("low_v", "HSV trackbar")
        high_h = cv2.getTrackbarPos("high_h", "HSV trackbar")
        high_s = cv2.getTrackbarPos("high_s", "HSV trackbar")
        high_v = cv2.getTrackbarPos("high_v", "HSV trackbar")
        point1_x = cv2.getTrackbarPos("point1_x", "HSV trackbar")
        point1_y = cv2.getTrackbarPos("point1_y", "HSV trackbar")
        point2_x = cv2.getTrackbarPos("point2_x", "HSV trackbar")
        point2_y = cv2.getTrackbarPos("point2_y", "HSV trackbar")
        point3_x = cv2.getTrackbarPos("point3_x", "HSV trackbar")
        point3_y = cv2.getTrackbarPos("point3_y", "HSV trackbar")
        point4_x = cv2.getTrackbarPos("point4_x", "HSV trackbar")
        point4_y = cv2.getTrackbarPos("point4_y", "HSV trackbar")
        camera_w_center = cv2.getTrackbarPos("camera_w_center", "HSV trackbar")
        return low_h, low_s, low_v, high_h, high_s, high_v,  point2_x, point2_y, point1_x, point1_y, point3_x, point3_y, point4_x, point4_y, camera_w_center

    def cameraCallback(self, mesaj):
        # get trackbar positions
        low_h, low_s, low_v, high_h, high_s, high_v, point1_x, point1_y, point2_x, point2_y, point3_x, point3_y, point4_x, point4_y, camera_w_center = self.getTrackbarPos()

        # low_h = 40
        # low_s = 90
        # low_v = 90
        # high_h = 110
        # high_s = 110
        # high_v = 110

        # convert the image to opencv format
        img = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")

        img_resized = cv2.resize(
            img, (0, 0), fx=0.5, fy=0.5)  # resize the image
        # crop the image
        crop_witdth = int(img_resized.shape[1] * 0.25)
        crop_height = int(img_resized.shape[0] * 0.40)
        corp_img = img_resized[crop_height-50:,
                               crop_witdth:img_resized.shape[1] - crop_witdth]

        self.line_track3(low_h, low_s, low_v, high_h,
                         high_s, high_v, point1_x, point1_y,
                         point2_x, point2_y, point3_x, point3_y,
                         point4_x, point4_y, camera_w_center, img_resized)

    def line_track1(self, low_h, low_s, low_v, high_h, high_s, high_v, img_resized):

        # convert the image to hsv format(hue,saturation,value)
        hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)

        # lower hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        lower = np.array([low_h, low_s, low_v])
        # upper hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        upper = np.array([high_h, high_s, high_v])

        mask = cv2.inRange(hsv, lower, upper)  # create a mask for the color
        # erode the mask to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        # dilate the mask to fill in gaps
        mask = cv2.dilate(mask, None, iterations=2)
        # blur the mask to make the edges clearer
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # bitwise and the image and the mask
        result = cv2.bitwise_and(img_resized, img_resized, mask=mask)
        h, w, d = img_resized.shape  # get the height,width and depth of the image
        rospy.loginfo("h: %d, w: %d, d: %d", h, w, d)
        # draw a circle at the center of the image to show the center of the image
        # (x,y,radius,color,thickness) -1 means filled ,color is BGR
        cv2.circle(img_resized, (int(w/2), int(h/2-20)), 5, (0, 0, 255), -1)
        M = cv2.moments(mask)  # find the moments of the mask
        if M['m00'] > 10:  # if m00 is not zero,m00 means the area of the image
            # cx is the x coordinate of the center of mass
            cx = int(M['m10']/M['m00'])
            # cy is the y coordinate of the center of mass
            cy = int(M['m01']/M['m00'])
            # draw a circle at the center of the contour to show the center of the contour
            cv2.circle(img_resized, (cx, cy), 5, (255, 0, 0), -1)
            # draw a line from the center of the contour to the center of the image
            cv2.line(img_resized, (cx-20, cy), (cx+20, cy), (255, 0, 0), 2)
            # draw a line from the center of the contour to the center of the image
            cv2.line(img_resized, (cx, cy-20), (cx, cy+20), (20, 100, 200), 2)
            # draw a line  for error correction
            cv2.line(img_resized, (cx-10, cy-10),
                     (cx-10, cy+10), (0, 255, 0), 2)  # (b, g, r)
            cv2.line(img_resized, (cx+10, cy-10),
                     (cx+10, cy+10), (0, 255, 0), 2)

            # calculate the distance from the center of the image to the center of the contour
            error = cx - w/2
            cv2.putText(img_resized, "Distance: {:.2f}cm".format(
                error), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            print(error)  # print the distance
            self.cmd_vel.linear.x = 0.4
            # set the angular velocity to the distance divided by 100
            self.cmd_vel.angular.z = -error/180
            self.pub.publish(self.cmd_vel)
        else:
            pass
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.pub.publish(self.cmd_vel)
        # put text on the image

        cv2.imshow("Orjinal", img_resized)
        cv2.imshow("Maske", mask)
        cv2.imshow("Sonuc", result)
        cv2.waitKey(1)  # wait for 1 ms

    def line_track2(self, low_h, low_s, low_v, high_h, high_s, high_v, img_resized):
        # convert the image to hsv format(hue,saturation,value)
        hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)

        # lower hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        lower = np.array([low_h, low_s, low_v])
        # upper hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        upper = np.array([high_h, high_s, high_v])

        mask = cv2.inRange(hsv, lower, upper)
        # erode the mask to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        # dilate the mask to fill in gaps
        mask = cv2.dilate(mask, None, iterations=2)
        # blur the mask to make the edges clearer
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # find the contours in the mask
        cnts = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # find the contours in the mask
        center = None  # initialize the center of the object
        if len(cnts) > 0:  # if there are contours
            c = max(cnts, key=cv2.contourArea)  # find the largest contour
            # find the center and radius of the largest contour
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # find the moments of the largest contour,moments mean the center of mass of the contour
            M = cv2.moments(c)
            # find the center of mass of the contour
            center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
            if radius > 10:  # if the radius is greater than 10
                # draw a circle around the contour
                cv2.circle(img_resized, (int(x), int(y)),
                           int(radius), (0, 255, 255), 2)
                # draw a circle at the center of the contour
                cv2.circle(img_resized, center, 5, (0, 0, 255), -1)
                self.cmd_vel.linear.x = 0.2  # set the linear velocity to 0.2
                self.cmd_vel.angular.z = 0.2  # set the angular velocity to 0.2
                self.pub.publish(self.cmd_vel)  # publish the command
        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.pub.publish(self.cmd_vel)
        cv2.imshow("mask", mask)  # show the mask
        cv2.imshow("image", img_resized)  # show the image
        cv2.waitKey(1)  # wait for 1 ms

    def line_track(self, low_h, low_s, low_v, high_h, high_s, high_v, img_resized):
        image_copy = img_resized.copy()
        # convert the image to hsv format(hue,saturation,value)
        hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)

        # lower hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        lower = np.array([low_h, low_s, low_v])
        # upper hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        upper = np.array([high_h, high_s, high_v])

        # create a mask for the color
        mask_range = cv2.inRange(hsv, lower, upper)
        # erode the mask to remove noise
        mask_dilate = cv2.erode(mask_range, None, iterations=3)
        # dilate the mask to fill in gaps
        mask_erode = cv2.dilate(mask_dilate, None, iterations=3)
        # blur the mask to make the edges clearer
        mask_blur = cv2.GaussianBlur(mask_erode, (5, 5), 0)

        # canny edge detection
        # (gray_img, low_threshold, high_threshold, apertureSize) apertureSize is the size of the Sobel kernel used to detect edges, it must be odd.
        cannyedges = cv2.Canny(mask_blur, 50, 150, apertureSize=3)

        # mask_inverse = cv2.bitwise_not(mask_blur)
        mask_inverse = mask_blur
        # bitwise and the image and the mask
        result_bitwise = cv2.bitwise_and(
            img_resized, img_resized, mask=mask_inverse)
        h, w, d = img_resized.shape  # get the height,width and depth of the image
        rospy.loginfo("h: %d, w: %d, d: %d", h, w, d)
        # draw a circle at the center of the image to show the center of the image
        # (x,y,radius,color,thickness) -1 means filled ,color is BGR

        # velocity tolerance coef
        linear_corefficient = 0.05
        angular_corefficient = 0.1

        # find the contours in the mask
        contours, hierarchy = read_camera.get_contours(
            mask_inverse, mod=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_NONE)

        # reference point for the line
        cv2.circle(result_bitwise, (int(w/2), int(h/2-h/3)),
                   5, (200, 0, 255), -1)  # color is BGR
        cv2.line(result_bitwise, (int(w/2), int(0)), (int(w/2), int(h)),
                 (0, 255, 0), 2, lineType=cv2.LINE_8)

        if len(contours) > 0:
            # find the largest contour in the mask
            c = max(contours, key=cv2.contourArea)
            # get the bounding rectangle of the largest contour
            bx, by, bw, bh = cv2.boundingRect(c)

            rect = cv2.minAreaRect(c)  # get the minimum area rectangle

            box = cv2.boxPoints(rect)  # get the four points of the rectangle
            # convert the points to integer, otherwise the points are float, and the line will be drawn in the wrong place
            box = np.int0(box)

            # get the center and size of the rectangle
            (rx, ry), (rh, rw), angle = rect

            image_angle = image_copy

            setpoint = w//2
            error = int(rx - setpoint)

            angle = int(angle)

            # cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
            # cv2.putText(image, str(ang), (10, 40),
            #         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # cv2.putText(image, str(error), (10, 320),
            #         cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # cv2.line(image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

            cv2.drawContours(image_angle, [box], 0, (0, 0, 255), 3)
            cv2.putText(image_angle, "angle "+str(angle), (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(image_angle, "error"+str(error), (10, 320),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # draw rx
            cv2.line(image_angle, (int(rx), 100),
                     (int(rx), 450), (255, 0, 0), 3)
            # draw setpoint
            cv2.line(image_angle, (int(setpoint), 10),
                     (int(setpoint), h-20), (0, 255, 0), 3)

            # for imageresixzed

            cv2.putText(img_resized,
                        "angle: %f " % (angle),
                        (w//8, h//4-50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  # (img , text , org , fontFace , fontScale , color , thickness , lineType , bottomLeftOrigin)
            # cv2.putText(img_resized, " rx: %f " % (rx), (w//4, h//4-20),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # cv2.putText(img_resized, " ry: %f " % (ry), (w//4, h//4+10),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "rw: %f " % (rw), (w//8, h//4+30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "rh: %f " % (rh), (w//8, h//4+50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # get the center of the rectangle
            centerx = int(rx + rw / 2)
            centery = int(ry + rh / 2)
            # get the center of the image
            imx = int(w / 2)
            imy = int(h / 2)
            # get the distance from the center of the image to the center of the rectangle
            distance_oclidian = math.sqrt(
                (imx - centerx)**2 + (imy - centery)**2)
            # get the angle of the rectangle
            angle_calculated = math.atan2(
                imy - centery, imx - centerx)  # in radians
            # get the angle of the rectangle in degrees
            angle_calculated_degrees = math.degrees(angle_calculated)

            # put  angle and distance tqqo the image
            cv2.putText(img_resized, "distance oclidian : %f " % (distance_oclidian),
                        (w//8, h//4+70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "angle: %f " % (angle_calculated_degrees),
                        (w//8, h//4+90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # get the linear and angular velocity
            linear_velocity = linear_corefficient * distance_oclidian
            angular_velocity = angular_corefficient * angle_calculated
            # put the linear and angular velocity to the image
            cv2.putText(img_resized, "linear velocity: %f " % (linear_velocity), (w//8, h//4+110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "angular velocity: %f " % (angular_velocity), (w//8, h//4+130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # publish the linear and angular velocity

            # draw the bounding rectangle
            cv2.drawContours(img_resized, [box], 0, (0, 255, 250), 2)

            # get the bounding rectangle of the largest contour
            cv2.drawContours(result_bitwise, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)  # find the moments of the mask
            if M['m00'] > 200:  # if m00 is not zero,m00 means the area of the image
                # cx is the x coordinate of the center of mass
                cx = int(M['m10']/M['m00'])
                # cy is the y coordinate of the center of mass
                cy = int(M['m01']/M['m00'])
                # draw a circle at the center of the contour to show the center of the contour
                cv2.circle(result_bitwise, (cx, cy), 5, (255, 0, 0), 2)

                # draw a line from the center of the contour to the reference point
                cv2.line(result_bitwise, (cx, cy),
                         (int(w/2), int(h/2-h/3)), (0, 0, 255), 2)
                # get the angle of the line from the center of the contour to the reference point
                angle_contour = math.atan2(int(h/2-h/3) - cy, int(w/2) - cx)
                distancex = int(w/2) - cx
                distancey = int(h/2-h/3) - cy
                # get the distance from the center of the contour to the reference point
                distance_contour = math.sqrt(distancex**2 + distancey**2)

                # put the angle and distance to the image
                cv2.putText(result_bitwise, "angle: %f " % (
                    angle_contour), (w//6, h//4-80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(result_bitwise, "distancex: %f " % (
                    distancex), (w//6, h//4-40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(result_bitwise, "distancey: %f " % (
                    distancey), (w//6, h//4-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(result_bitwise, "distance: %f " % (
                    distance_contour), (w//6, h//4+20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                cv2.line(result_bitwise, (cx-20, cy),
                         (cx+20, cy), (255, 0, 0), 2)

                cv2.line(result_bitwise, (cx, cy-20),
                         (cx, cy+20), (20, 100, 200), 2)
                # draw a line  for error correction
                cv2.line(result_bitwise, (cx-10, cy-10),
                         (cx-10, cy+10), (0, 255, 0), 2)  # (b, g, r)
                cv2.line(result_bitwise, (cx+10, cy-10),
                         (cx+10, cy+10), (0, 255, 0), 2)

                # calculate the distance from the center of the image to the center of the contour
                error = cx - w/2
                cv2.putText(result_bitwise, "Error: {:.2f}cm".format(
                    error), (w//4, h//4+40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                print(error)  # print the distance
                self.cmd_vel.linear.x = 1 * linear_corefficient
                # set the angular velocity to the distance divided by 100
                self.cmd_vel.angular.z = -error/30000*angular_corefficient
                self.pub.publish(self.cmd_vel)
            else:
                pass
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.pub.publish(self.cmd_vel)

            img_labels = [["img resized", "hsv img", "mask_inrange", "image_angle"],
                          ["mask_erode", "mask_blur", "cannyedges", "result_bitwise"]]

            img_list = ([img_resized, hsv, mask_inverse, image_angle], [
                mask_erode, mask_blur, cannyedges, result_bitwise])

            stack_img = utl.stackImageslabels(img_list, img_labels, 1)
            cv2.imshow("stack", stack_img)
        # cv2.waitKey(1)  # wait for 1 ms before moving to the next frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    def line_track3(self, low_h, low_s, low_v, high_h,
                    high_s, high_v, point1_x, point1_y,
                    point2_x, point2_y, point3_x, point3_y,
                    point4_x, point4_y, camera_w_center, img_resized):

        # create trapezoid mask for the image
        blank_roi = np.zeros_like(img_resized)

        roi = img_resized[point1_y:point2_y, point1_x:point2_x]
        if roi.size != 0:
            cv2.imshow("roi", roi)

        roi_trapezoid = np.array([[point1_x, point1_y], [point2_x, point2_y],
                                  [point3_x, point3_y], [point4_x, point4_y]], np.int32)
        # draw the trapezoid on the blank image
        cv2.polylines(blank_roi, [roi_trapezoid], True, (255, 255, 255), 2)
        # fill the trapezoid with white color
        cv2.fillPoly(blank_roi, [roi_trapezoid], (255, 255, 255))
        # bitwise and between the blank image and the original image
        mask_trapezoid = cv2.bitwise_and(img_resized, blank_roi)

        image_copy = img_resized.copy()
        # convert the image to hsv format(hue,saturation,value)
        hsv = cv2.cvtColor(mask_trapezoid, cv2.COLOR_BGR2HSV)

        # lower hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        lower = np.array([low_h, low_s, low_v])
        # upper hsv values for the color,hue is the color,saturation is the intensity of the color,value is the brightness of the color
        upper = np.array([high_h, high_s, high_v])

        # create a mask for the color
        mask_range = cv2.inRange(hsv, lower, upper)
        # erode the mask to remove noise
        mask_dilate = cv2.erode(mask_range, None, iterations=3)
        # dilate the mask to fill in gaps
        mask_erode = cv2.dilate(mask_dilate, None, iterations=3)
        # blur the mask to make the edges clearer
        mask_blur = cv2.GaussianBlur(mask_erode, (5, 5), 0)

        # canny edge detection
        # (gray_img, low_threshold, high_threshold, apertureSize) apertureSize is the size of the Sobel kernel used to detect edges, it must be odd.
        cannyedges = cv2.Canny(mask_blur, 50, 150, apertureSize=3)

        # bitwise and the image and the mask
        result_bitwise = cv2.bitwise_and(
            img_resized, img_resized, mask=mask_blur)

        h, w, d = img_resized.shape  # get the height,width and depth of the image
        rospy.loginfo("h: %d, w: %d, d: %d", h, w, d)

        # draw a circle at the center of the image to show the center of the image
        # (x,y,radius,color,thickness) -1 means filled ,color is BGR

        # velocity tolerance coef
        linear_corefficient = 0.15
        angular_corefficient = 1.5

        # find the contours in the mask
        contours, hierarchy = read_camera.get_contours(
            mask_blur, mod=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_NONE)

        # reference point for the line
        cv2.circle(result_bitwise, (int(camera_w_center), int(h/2-h/3)),
                   5, (200, 0, 255), -1)  # color is BGR
        cv2.line(result_bitwise, (int(camera_w_center), int(0)), (int(camera_w_center), int(h)),
                 (0, 255, 0), 2, lineType=cv2.LINE_8)

        if len(contours) > 0:
            # find the largest contour in the mask
            c = max(contours, key=cv2.contourArea)
            # get the bounding rectangle of the largest contour
            bx, by, bw, bh = cv2.boundingRect(c)

            rect = cv2.minAreaRect(c)  # get the minimum area rectangle

            box = cv2.boxPoints(rect)  # get the four points of the rectangle
            # convert the points to integer, otherwise the points are float, and the line will be drawn in the wrong place
            box = np.int0(box)

            # get the center and size of the rectangle
            (rx, ry), (rh, rw), angle = rect

            image_angle = image_copy

            setpoint = camera_w_center  # setpoint is the center of the image
            error = int(rx - setpoint)

            angle = int(angle)

            # cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
            # cv2.putText(image, str(ang), (10, 40),
            #         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # cv2.putText(image, str(error), (10, 320),
            #         cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # cv2.line(image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

            cv2.drawContours(image_angle, [box], 0, (0, 0, 255), 3)
            cv2.putText(image_angle, "angle "+str(angle), (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(image_angle, "error"+str(error), (10, 320),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # draw rx
            cv2.line(image_angle, (int(rx), 100),
                     (int(rx), 450), (255, 0, 0), 3)
            # draw setpoint
            cv2.line(image_angle, (int(setpoint), 10),
                     (int(setpoint), h-20), (0, 255, 0), 3)

            # for imageresixzed

            cv2.putText(img_resized,
                        "angle: %f " % (angle),
                        (w//8, h//4-50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  # (img , text , org , fontFace , fontScale , color , thickness , lineType , bottomLeftOrigin)
            # cv2.putText(img_resized, " rx: %f " % (rx), (w//4, h//4-20),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # cv2.putText(img_resized, " ry: %f " % (ry), (w//4, h//4+10),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "rw: %f " % (rw), (w//8, h//4+30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "rh: %f " % (rh), (w//8, h//4+50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # get the center of the rectangle
            centerx = int(rx + rw / 2)
            centery = int(ry + rh / 2)
            # get the center of the image
            imx = int(w / 2)
            imy = int(h / 2)
            # get the distance from the center of the image to the center of the rectangle
            distance_oclidian = math.sqrt(
                (imx - centerx)**2 + (imy - centery)**2)
            # get the angle of the rectangle
            angle_calculated = math.atan2(
                imy - centery, imx - centerx)  # in radians
            # get the angle of the rectangle in degrees
            angle_calculated_degrees = math.degrees(angle_calculated)

            # put  angle and distance tqqo the image
            cv2.putText(img_resized, "distance oclidian : %f " % (distance_oclidian),
                        (w//8, h//4+70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "angle: %f " % (angle_calculated_degrees),
                        (w//8, h//4+90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # get the linear and angular velocity
            linear_velocity = linear_corefficient * distance_oclidian
            angular_velocity = angular_corefficient * angle_calculated
            # put the linear and angular velocity to the image
            cv2.putText(img_resized, "linear velocity: %f " % (linear_velocity), (w//8, h//4+110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(img_resized, "angular velocity: %f " % (angular_velocity), (w//8, h//4+130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            # publish the linear and angular velocity

            # draw the bounding rectangle
            cv2.drawContours(img_resized, [box], 0, (0, 255, 250), 2)

            # get the bounding rectangle of the largest contour
            cv2.drawContours(result_bitwise, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)  # find the moments of the mask
            if M['m00'] > 200:  # if m00 is not zero,m00 means the area of the image
                # cx is the x coordinate of the center of mass
                cx = int(M['m10']/M['m00'])
                # cy is the y coordinate of the center of mass
                cy = int(M['m01']/M['m00'])
                # draw a circle at the center of the contour to show the center of the contour
                cv2.circle(result_bitwise, (cx, cy), 5, (255, 0, 0), 2)

                # draw a line from the center of the contour to the reference point
                cv2.line(result_bitwise, (cx, cy),
                         (int(camera_w_center), int(h/2-h/3)), (0, 0, 255), 2)
                # get the angle of the line from the center of the contour to the reference point
                angle_contour = math.atan2(
                    int(h/2-h/3) - cy, int(camera_w_center) - cx)
                distancex = int(camera_w_center) - cx
                distancey = int(h/2-h/3) - cy
                # get the distance from the center of the contour to the reference point
                distance_contour = math.sqrt(distancex**2 + distancey**2)

                # put the angle and distance to the image
                cv2.putText(result_bitwise, "angle: %f " % (
                    angle_contour), (w//6, h//4-80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(result_bitwise, "distancex: %f " % (
                    distancex), (w//6, h//4-40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(result_bitwise, "distancey: %f " % (
                    distancey), (w//6, h//4-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(result_bitwise, "distance: %f " % (
                    distance_contour), (w//6, h//4+20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                cv2.line(result_bitwise, (cx-20, cy),
                         (cx+20, cy), (255, 0, 0), 2)

                cv2.line(result_bitwise, (cx, cy-20),
                         (cx, cy+20), (20, 100, 200), 2)
                # draw a line  for error correction
                cv2.line(result_bitwise, (cx-10, cy-10),
                         (cx-10, cy+10), (0, 255, 0), 2)  # (b, g, r)
                cv2.line(result_bitwise, (cx+10, cy-10),
                         (cx+10, cy+10), (0, 255, 0), 2)

                # calculate the distance from the center of the image to the center of the contour
                error = cx - camera_w_center
                cv2.putText(result_bitwise, "Error: {:.2f}cm".format(
                    error), (w//4, h//4+40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                print(error)  # print the distance
                self.cmd_vel.linear.x = 1 * linear_corefficient
                # set the angular velocity to the distance divided by 100
                self.cmd_vel.angular.z = -error/600*angular_corefficient
                self.pub.publish(self.cmd_vel)
            else:
                pass
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.pub.publish(self.cmd_vel)

            cv2.putText(result_bitwise, "cmd_linear_x: %f " % (self.cmd_vel.linear.x), (w//8, h//4+110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(result_bitwise, "cmd_angular_z: %f " % (self.cmd_vel.angular.z), (w//8, h//4+130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            img_labels = [["blank_roi", "img resized", "hsv img", "mask_inrange", "image_angle"],
                          ["mask_roi", "mask_erode", "mask_blur", "cannyedges", "result_bitwise"]]

            img_list = ([blank_roi, img_resized, hsv, mask_range, image_angle], [
                        mask_trapezoid, mask_erode, mask_blur, cannyedges, result_bitwise])

            stack_img = utl.stackImageslabels(img_list, img_labels, 0.5)
            cv2.imshow("stack", stack_img)
        # cv2.waitKey(1)  # wait for 1 ms before moving to the next frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


if __name__ == "__main__":
    LineTrack()
    rospy.spin()
