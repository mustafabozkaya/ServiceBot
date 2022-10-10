import cv2
import numpy as np


def select_roi():

    # image_path
    img_path = "./img/img.jpg"

    # read image
    img_raw = cv2.imread(img_path)

    # select ROI function
    roi = cv2.selectROI(img_raw)

    # print rectangle points of selected roi
    print(roi)

    # Crop selected roi from raw image
    roi_cropped = img_raw[int(roi[1]):int(roi[1]+roi[3]),
                          int(roi[0]):int(roi[0]+roi[2])]

    # show cropped image
    cv2.imshow("ROI", roi_cropped)

    cv2.imwrite("crop.jpeg", roi_cropped)

    # hold window
    cv2.waitKey(0)


def select_rois():

    # image_path
    img_path = "./img/img.jpg"

    # read image
    img_raw = cv2.imread(img_path)

    # select ROIs function
    ROIs = cv2.selectROIs("Select Rois", img_raw)

    # print rectangle points of selected roi
    print(ROIs)

    # Crop selected roi ffrom raw image

    # counter to save image with different name
    crop_number = 0

    # loop over every bounding box save in array "ROIs"
    for rect in ROIs:
        # crop image
        x1 = rect[0]
        y1 = rect[1]
        x2 = rect[2]
        y2 = rect[3]

        # crop roi from original image
        img_crop = img_raw[y1:y1+y2, x1:x1+x2]

        # show cropped image
        cv2.imshow("crop"+str(crop_number), img_crop)

        # save cropped image
        cv2.imwrite("crop"+str(crop_number)+".jpeg", img_crop)

        crop_number += 1

    # hold window
    cv2.waitKey(0)


if __name__ == "__main__":
    select_rois()
