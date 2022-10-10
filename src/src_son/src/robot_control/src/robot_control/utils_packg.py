import sys
import cv2 as cv
import time
import numpy as np
import pandas as pd





def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor_id=0 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
# gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
#    'video/x-raw(memory:NVMM),width=3280, height=2464, framerate=21/1, format=NV12' ! \
#    nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=720' ! \
#    nvvidconv ! nvegltransform ! nveglglessink -e

# Python code to check for empty list
# Explicit way


def Enquiry(lis1):
    if len(lis1) == 0:
        return 0
    else:
        return 1

# TO STACK ALL THE IMAGES IN ONE WINDOW


def stackImages(imgArray, scale=0.5):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv.resize(
                        imgArray[x][y], None, scale, scale, interpolation=cv.INTER_CUBIC)
                else:
                    imgArray[x][y] = cv.resize(
                        imgArray[x][y], None, scale, scale, interpolation=cv.INTER_CUBIC)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv.cvtColor(
                        imgArray[x][y], cv.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv.resize(
                    imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv.resize(
                    imgArray[x],  None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv.cvtColor(imgArray[x], cv.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver
# TO STACK ALL THE IMAGES IN ONE WINDOW


def stackImageslabels(imgArray, lables=[], scale=0.5):
    rows = len(imgArray)
    cols = len(imgArray[0])
    # print(rows, cols)
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    # print("height:", height, "width:", width)
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                imgArray[x][y] = cv.resize(
                    imgArray[x][y], (0, 0), fx=scale, fy=scale, interpolation=cv.INTER_CUBIC)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv.cvtColor(
                        imgArray[x][y], cv.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
            hor_con[x] = np.concatenate(imgArray[x])
        ver = np.vstack(hor)
        ver_con = np.concatenate(hor)
    else:
        for x in range(0, rows):
            imgArray[x] = cv.resize(
                imgArray[x], (0, 0), fx=scale, fy=scale, interpolation=cv.INTER_AREA)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv.cvtColor(imgArray[x], cv.COLOR_GRAY2BGR)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv.cvtColor(imgArray[x], cv.COLOR_GRAY2BGR)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv.cvtColor(imgArray[x], cv.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        hor_con = np.concatenate(imgArray)
        ver = hor
    if Enquiry(lables):
        eachImgWidth = int(ver.shape[1] / cols)
        eachImgHeight = int(ver.shape[0] / rows)
        # print(eachImgHeight)
        for d in range(0, rows):
            for c in range(0, cols):
                cv.rectangle(ver, (c * eachImgWidth, eachImgHeight * d),
                             (c * eachImgWidth + len(lables[d][c]) * 13 +
                              27, 30 + eachImgHeight * d), (255, 255, 255),
                             cv.FILLED)
                cv.putText(ver, lables[d][c], (eachImgWidth * c + 10, eachImgHeight * d + 20),
                           cv.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 255), 2)
    return ver


def reorder(myPoints):
    print(myPoints, myPoints.shape)
    myPoints = myPoints.reshape((4, 2))
    print(myPoints, myPoints.shape)
    myPointsNew = np.zeros((4, 1, 2), dtype=np.int32)
    add = myPoints.sum(1)
    # add0 = myPoints.sum(0)
    # add2 = myPoints.sum(2)
    # add_ = myPoints.sum()
    # add3 = myPoints.sum(3)

    myPointsNew[0] = myPoints[np.argmin(add)]
    myPointsNew[2] = myPoints[np.argmax(add)]
    # myPoints = np.delete(myPoints, np.argmin(add), axis=0)
    # myPoints=np.delete(myPoints,np.argmax(add),axis=0)

    diff = np.diff(myPoints, axis=1)
    myPointsNew[1] = myPoints[np.argmin(diff)]
    myPointsNew[3] = myPoints[np.argmax(diff)]

    return myPointsNew


def order_points(pts):
    pts = pts.reshape((4, 2))
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 1, 2), dtype="float32")
    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # return the ordered coordinates
    return rect


def four_point_transform(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")
    # compute the perspective transform matrix and then apply it
    M = cv.getPerspectiveTransform(rect, dst)
    warped = cv.warpPerspective(image, M, (maxWidth, maxHeight))
    # return the warped image
    return warped


def Contourarea(contours):
    biggest = np.array([])
    max_area = 0
    for i in contours:
        print(contours)
        area = cv.contourArea(i)
        if area > 50 and area < 100:
            peri = cv.arcLength(i, True)
            approx = cv.approxPolyDP(i, 0.02 * peri, True)
            if area > max_area and len(approx) == 4:
                np.append(biggest, approx)
                max_area = area
    return biggest, max_area


def getContours(imgDil, imgConturs, Area, imgQr, X_Y, pixel_mean):
    contours, hierarchy = cv.findContours(
        imgDil, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv.contourArea(cnt)
        #print(f"contur points -----{len(cnt)}")
        # print(Area)
        if area > Area[0] and area < Area[1]:
            #cv.drawContours(imgConturs, cnt, -1, (255, 20, 0), 2)
            imgConturs.shape
            peri = cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, 0.01*peri, True)
            #print(f"approx points -----{len(approx)}")
            (x, y, w, h) = cv.boundingRect(approx)

            if h <= w <= 1.4*h and X_Y[2] > x > X_Y[0] and X_Y[1] < y < X_Y[3]:
                cropimg = imgQr[y:y+h, x:x+w]

                croparray = np.asarray(cropimg).flatten()
                if pixel_mean[0] < croparray.mean() < pixel_mean[1]:
                    cv.imshow("Qrlar", cropimg)
                    print(
                        f"croparray.mean - {croparray.mean()} - croparray.max - {croparray.max()} - croparray.min -- {croparray.min()}")
                    cv.rectangle(imgConturs, (x, y),
                                 (x+w, y+h), (230, 200, 120), 4)
                    cv.putText(imgConturs, "CNT point  number ::"+str(len(approx)),
                               (x-30, y-10), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 250), 2)
                    cv.putText(imgConturs, "approxx Points number ::"+str(len(approx)),
                               (x-30, y-25), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 250), 2)
                    cv.putText(imgConturs, "Area ::"+str(cv.contourArea(approx)),
                               (x-30, y-40), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 250), 2)


def drawRectangle(img, biggest, thickness):
    cv.line(img, (biggest[0][0][0], biggest[0][0][1]),
            (biggest[1][0][0], biggest[1][0][1]), (0, 0, 255), thickness)
    cv.line(img, (biggest[0][0][0], biggest[0][0][1]),
            (biggest[2][0][0], biggest[2][0][1]), (0, 0, 255), thickness)
    cv.line(img, (biggest[3][0][0], biggest[3][0][1]),
            (biggest[2][0][0], biggest[2][0][1]), (0, 0, 255), thickness)
    cv.line(img, (biggest[3][0][0], biggest[3][0][1]),
            (biggest[1][0][0], biggest[1][0][1]), (0, 0, 255), thickness)

    return img


def passFunction(num):
    pass


def on_trackbar(val):
    kernel = pd.DataFrame(
        ["cv.MORPH_RECT", "cv.MORPH_ELLIPSE", "cv.MORPH_CROSS"])
    for value in kernel.index:
        if val == value:
            return kernel._get_value(val, 0)
        else:
            return cv.MORPH_RECT


def kernelsize(val):
    if val % 2:
        return val
    else:
        val += 1
        return val

    cv.imshow(title_window, dst)


def initializeTrackbars(intialTracbarVals=0):
    cv.namedWindow("parameters")
    cv.resizeWindow("parameters", 500, 700)
    cv.createTrackbar("threshold", "parameters", 50, 255, passFunction)
    cv.createTrackbar("kernelsize", "parameters", 2, 15, passFunction)
    cv.createTrackbar("kerneltype", "parameters", 0, 2, on_trackbar)
    cv.createTrackbar("Maxthreshold", "parameters", 88, 255, passFunction)
    cv.createTrackbar("Areamin", "parameters", 2000, 30000, passFunction)
    cv.createTrackbar("Areamax", "parameters", 6000, 30000, passFunction)
    cv.createTrackbar("Xmax", "parameters", 55, 650, passFunction)
    cv.createTrackbar("Xmin", "parameters", 80, 650, passFunction)
    cv.createTrackbar("Ymax", "parameters", 500, 570, passFunction)
    cv.createTrackbar("Ymin", "parameters", 80, 570, passFunction)
    cv.createTrackbar("pixel_max", "parameters", 118, 255, passFunction)
    cv.createTrackbar("pixel_min", "parameters", 30, 255, passFunction)
    cv.createTrackbar("left_right", "parameters", 30, 1000, passFunction)
    cv.createTrackbar("top_bottom", "parameters", 30, 1000, passFunction)


def valTrackbars():
    thres1 = cv.getTrackbarPos("threshold", "parameters")
    thres2 = cv.getTrackbarPos("Maxthreshold", "parameters")
    areamin = cv.getTrackbarPos("Areamin", "parameters")
    areamax = cv.getTrackbarPos("Areamax", "parameters")
    Xmax = cv.getTrackbarPos("Xmax", "parameters")
    Xmin = cv.getTrackbarPos("Xmin", "parameters")
    Ymax = cv.getTrackbarPos("Ymax", "parameters")
    Ymin = cv.getTrackbarPos("Ymin", "parameters")
    pixel_max = cv.getTrackbarPos("pixel_max", "parameters")
    pixel_min = cv.getTrackbarPos("pixel_min", "parameters")
    src = (thres1, thres2, areamax, areamin, Xmax,
           Xmin, Ymax, Ymin, pixel_max, pixel_min)
    return src
