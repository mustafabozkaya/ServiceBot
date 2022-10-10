#!/usr/bin/env python3

import numpy as np
import cv2

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)


def background_subtraction(history, learning_rate, detect_shadows=False):
    # create a background subtractor
    # MOG2 is a gaussian Mixture-of-Gaussians Background Subtractor
    # args: history, varThreshold, bgThreshold, bgRatio, fgThreshold, learningRate, varThreshold, detectShadows
    # history is the number of frames to use to train the background model
    # varThreshold is the maximum allowed difference between the current frame and the background model
    # bgThreshold is the minimum difference between the current frame and the background model
    # bgRatio is the maximum difference between the current frame and the background model
    # fgThreshold is the minimum difference between the current frame and the background model
    # learningRate is the rate at which the background model is updated
    # detectShadows is a boolean that determines whether or not shadows are detected

    # create background subtractor object using MOG2 algorithm , fbgb is the foreground background object which is used to subtract the background from the image and get the foreground image
    fbgb = cv2.createBackgroundSubtractorMOG2(history=history, varThreshold=learning_rate, bgThreshold=learning_rate,
                                              bgRatio=learning_rate, learning_rate=learning_rate, detectShadows=detect_shadows)
    return fbgb


def split_rgb(img):
    # resize the image to a smaller size
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    b, g, r = cv2.split(img)  # get b,g,r
    # b, g, r = np.split(img, 3)  # "split" splits the image into 3 channels
    img_rgb = np.concatenate((r, g, b), axis=0)
    cv2.imshow("img-rgb", img_rgb)


def draw_rectangle(img):
    # resize the image to a smaller size
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    # draw a rectangle
    # img, pt1, pt2, color, thickness,pt1 is the top left corner and pt2 is the bottom right corner
    cv2.rectangle(img, (0, 0), (100, 100), (0, 255, 0), 2)
    cv2.imshow("img-rectangle", img)


def draw_circle(img):
    # resize the image to a smaller size
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    # draw a circle
    # img, center, radius, color, thickness
    cv2.circle(img, (100, 100), 50, (0, 255, 0), 2)
    cv2.imshow("img-circle", img)


def draw_line(img):
    # resize img
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    # draw a line
    # img, pt1, pt2, color, thickness
    cv2.line(img, (0, 0), (100, 100), (0, 255, 0), 2)


def draw_text(img):
    # resize img
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    # draw a text
    # img, text, org, fontFace, fontScale, color, thickness
    cv2.putText(img, "Hello World", (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("img-text", img)


def draw_polygon(img):
    # resize img
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    # draw a polygon
    pts = np.array([[10, 5], [20, 30], [70, 20], [50, 10]],
                   np.int32)  # pts is a numpy array of points
    print(pts.shape)
    pts = pts.reshape((-1, 1, 2))  # pts is a numpy array of points
    print(pts.shape)
    # img, pts, isClosed, color, thickness
    cv2.polylines(img, [pts], True, (0, 255, 0), 2)
    cv2.imshow("img-polygon", img)


def convert_gray(img):
    # convert to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    cv2.imshow("img-gray", img)
    return img


def basic_threshold(grayimg, threshold, max_value, type):

    # threshold the image
    # img, thresh, maxval, type(cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV, cv2.THRESH_TRUNC, cv2.THRESH_TOZERO, cv2.THRESH_TOZERO_INV)
    ret, thresh = cv2.threshold(grayimg, thresh, max_value, type)
    cv2.imshow("img-threshold", thresh)


def adaptive_threshold(img, max_value, adaptive_method, threshold_type, block_size=(3, 3), constant=0):

    # adaptive threshold the image
    # img, max_value, adaptive_method, threshold_type, block_size, constant
    # max_value is the value to which the pixels are set to if the condition is met (255 for binary, 0 for binary inverted)
    # adaptive_method is the type of thresholding algorithm to use (cv2.ADAPTIVE_THRESH_MEAN_C, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
    # threshold_type is the thresholding algorithm to use (cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV, cv2.THRESH_TRUNC, cv2.THRESH_TOZERO, cv2.THRESH_TOZERO_INV)
    # block_size is the size of a pixel neighborhood that is used to calculate a threshold value for the pixel (3, 5, 7, and so on)
    # constant is the constant subtracted from the mean or weighted mean (usually set to 0)
    thresh = cv2.adaptiveThreshold(
        img, max_value, adaptive_method, threshold_type, block_size, constant)

    cv2.imshow("img-adaptive-threshold", thresh)


def inrange(img, lower, upper):
    # convert to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    # inrange the image
    # img, lower, upper
    # lower is the lower bound of the range to be included in the output image
    # upper is the upper bound of the range to be included in the output image
    thresh = cv2.inRange(img, lower, upper)
    cv2.imshow("img-inrange", thresh)


def inrange_hsv(img, lower, upper):
    # convert to hsv
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert to hsv
    # inrange the image
    # img, lower, upper
    # lower is the lower bound of the range to be included in the output image
    # upper is the upper bound of the range to be included in the output image
    thresh = cv2.inRange(img, lower, upper)
    cv2.imshow("img-inrange-hsv", thresh)


def canny_edge_detection(img, threshold1, threshold2):
    # convert to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    # canny edge detection
    # img, threshold1, threshold2, apertureSize, L2gradient
    # threshold1 is the lower threshold for the hysteresis procedure
    # threshold2 is the upper threshold for the hysteresis procedure
    # apertureSize is the size of the Sobel aperture, which is used to calculate the gradient magnitude (3, 5, 7, and so on)
    # L2gradient is a flag, which indicates whether a more accurate L2 norm is used to calculate the image gradient magnitude (cv2.L2gradient)
    edges = cv2.Canny(img, threshold1, threshold2)
    cv2.imshow("img-canny-edge-detection", edges)


def laplacian_edge_detection(img):
    # convert to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    # laplacian edge detection
    # img, apertureSize, L2gradient
    # apertureSize is the size of the Sobel aperture, which is used to calculate the gradient magnitude (3, 5, 7, and so on)
    # L2gradient is a flag, which indicates whether a more accurate L2 norm is used to calculate the image gradient magnitude (cv2.L2gradient)
    edges = cv2.Laplacian(img, cv2.CV_64F)
    cv2.imshow("img-laplacian-edge-detection", edges)


def sobel_edge_detection(img, x_order, y_order):
    # convert to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    # sobel edge detection
    # img, x_order, y_order, ksize, L2gradient
    # x_order is the order of the derivative x
    # y_order is the order of the derivative y
    # ksize is the size of the Sobel kernel
    # L2gradient is a flag, which indicates whether a more accurate L2 norm is used to calculate the image gradient magnitude (cv2.L2gradient)
    edges = cv2.Sobel(img, cv2.CV_64F, x_order, y_order, ksize=3)
    cv2.imshow("img-sobel-edge-detection", edges)


def laplacian_edge_detection_hsv(img):
    # convert to hsv
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert to hsv
    # laplacian edge detection
    # img, apertureSize, L2gradient
    # apertureSize is the size of the Sobel aperture, which is used to calculate the gradient magnitude (3, 5, 7, and so on)
    # L2gradient is a flag, which indicates whether a more accurate L2 norm is used to calculate the image gradient magnitude (cv2.L2gradient)
    edges = cv2.Laplacian(img, cv2.CV_64F)
    cv2.imshow("img-laplacian-edge-detection-hsv", edges)


def sobel_edge_detection_hsv(img, x_order, y_order):
    # convert to hsv
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert to hsv
    # sobel edge detection
    # img, x_order, y_order, ksize, L2gradient
    # x_order is the order of the derivative x
    # y_order is the order of the derivative y
    # ksize is the size of the Sobel kernel
    # L2gradient is a flag, which indicates whether a more accurate L2 norm is used to calculate the image gradient magnitude (cv2.L2gradient)
    edges = cv2.Sobel(img, cv2.CV_64F, x_order, y_order, ksize=3)
    cv2.imshow("img-sobel-edge-detection-hsv", edges)


def laplacian_edge_detection_hsv_inrange(img, lower, upper):
    # convert to hsv
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert to hsv
    # inrange the image
    # img, lower, upper
    # lower is the lower bound of the range to be included in the output image
    # upper is the upper bound of the range to be included in the output image
    thresh = cv2.inRange(img, lower, upper)
    # laplacian edge detection
    # img, apertureSize, L2gradient
    # apertureSize is the size of the Sobel aperture, which is used to calculate the gradient magnitude (3, 5, 7, and so on)
    # L2gradient is a flag, which indicates whether a more accurate L2 norm is used to calculate the image gradient magnitude (cv2.L2gradient)
    edges = cv2.Laplacian(thresh, cv2.CV_64F)
    cv2.imshow("img-laplacian-edge-detection-hsv-inrange", edges)


def hough_line_detection(img):
    # convert to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    # hough line detection
    # img, rho, theta, threshold, minLineLength, maxLineGap
    # rho is the distance resolution of the accumulator in pixels
    # theta is the angle resolution of the accumulator in radians
    # threshold is the accumulator threshold parameter
    # minLineLength is the minimum number of points that can form a line
    # maxLineGap is the maximum gap between two points that can be considered to form a line
    lines = cv2.HoughLines(img, 1, np.pi / 180, 100)
    # draw the lines
    for rho, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
    cv2.imshow("img-hough-line-detection", img)


def get_contours(grayimg, threshold, max_value, type):

    # img, mode(cv2.RETR_LIST, cv2.RETR_EXTERNAL, cv2.RETR_TREE),method (cv2.CHAIN_APPROX_SIMPLE, cv2.CHAIN_APPROX_NONE)
    contours, hierarchy = cv2.findContours(
        grayimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # img, mode(cv2.RETR_LIST, cv2.RETR_EXTERNAL, cv2.RETR_TREE),method (cv2.CHAIN_APPROX_SIMPLE, cv2.CHAIN_APPROX_NONE)
    # img, contours, contourIdx, color, thickness
    cv2.drawContours(img, contours, -1, (0, 255, 0),
                     2)  # -1 means all contours
    cv2.imshow("img-contours", img)
    return contours


def draw_contours(img, contours):

    cv2.drawContours(img, contours, -1, (0, 255, 0),
                     2)  # -1 means all contours
    cv2.imshow("img-contours", img)


def process_contours(binary_img, img, contours):
    # create a black image
    black_img = np.zeros(img.shape[0], img.shape[1], np.uint8)
    for c in contours:

        # get area of the contour
        area = cv2.contourArea(c)
        # get the perimeter of the contour
        perimeter = cv2.arcLength(c, True)  # True means closed contour
        # get the equivalent diameter
        diameter = np.sqrt(4 * area / np.pi)
        # get the equivalent radius
        radius = diameter / 2
        # get the minimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(c)
        # get the minimum enclosing rectangle
        x, y, w, h = cv2.boundingRect(c)
        # draw the bounding rect
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # draw the contour
        cv2.drawContours(black_img, [c], -1, (255, 255, 255), -1)
        # get the mask
        mask = np.zeros(binary_img.shape, np.uint8)
        cv2.drawContours(mask, [c], -1, 255, -1)
        # get the mean
        mean = cv2.mean(img, mask)
        # draw the mean
        cv2.circle(img, (int(mean[0]), int(mean[1])), 5, (0, 0, 255), -1)
        # draw the mean in the black image
        cv2.circle(black_img, (int(mean[0]), int(
            mean[1])), 5, (255, 255, 255), -1)

    print("number of counts:", str(len(contours))


def get_contour_center(contour):
    # get the center of the contour
    # get the moments of the contour (area, mass center, etc.)
    M=cv2.moments(contour)
    if M["m00"] != 0:
        # get the x coordinate of the center of the contour
        cx=int(M["m10"] / M["m00"])
        # get the y coordinate of the center of the contour
        cy=int(M["m01"] / M["m00"])
        return cx, cy


def get_moments(contour):
    # get the moments of the contour
    # inside M dictionary, there are the following keys:
    # "m00": area of the contour
    # "m10": x coordinate of the mass center of the contour
    # "m01": y coordinate of the mass center of the contour
    # "m02": x coordinate of the second central moment of the contour
    # "m11": y coordinate of the second central moment of the contour
    # "m12": x coordinate of the third central moment of the contour
    # "m21": y coordinate of the third central moment of the contour
    # "m20": x coordinate of the fourth central moment of the contour
    # "m02": y coordinate of the fourth central moment of the contour
    # "m30": x coordinate of the fifth central moment of the contour
    # "m03": y coordinate of the fifth central moment of the contour
    # "m40": x coordinate of the sixth central moment of the contour
    # "m04": y coordinate of the sixth central moment of the contour
    # "m50": x coordinate of the seventh central moment of the contour
    # "m05": y coordinate of the seventh central moment of the contour
    # "m60": x coordinate of the eighth central moment of the contour
    # "m06": y coordinate of the eighth central moment of the contour
    # "m61": x coordinate of the ninth central moment of the contour
    # "m07": y coordinate of the ninth central moment of the contour
    # "m62": x coordinate of the tenth central moment of the contour
    # "m08": y coordinate of the tenth central moment of the contour
    # get the moments of the contour (area, mass center, etc.)
    M=cv2.moments(contour)
    # M is a dictionary of the moments of the contour (area, mass center, etc.)
    return M


def draw_all(img):
    draw_rectangle(img)
    draw_circle(img)
    draw_line(img)
    draw_text(img)
    draw_polygon(img)
    draw_contours(img)
    split_rgb(img)


while(True):
    # Capture frame-by-frame
    # read gray scale image
    ret, frame=cap.read(cv2.IMREAD_GRAYSCALE)
    # ret, frame = cap.read()
    # draw_all(frame)
    get_moments(convert_gray(frame))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
