#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

# Load calibration data
with open('/home/nvidia/xycar_ws/src/lane_detection_project/config/ost.yaml') as f:
    loadeddict = yaml.safe_load(f)
mtx = np.reshape(np.array(loadeddict['camera_matrix']['data']), (loadeddict['camera_matrix']['rows'], loadeddict['camera_matrix']['cols']))
print("mtx : ", mtx)
dist = np.array(loadeddict['distortion_coefficients']['data'])

frame_num = 0

def callback(data):

    height, width = data.height, data.width
    encoding = 'rgb8' if 'rgb8' in data.encoding else data.encoding
    if encoding == 'rgb8':
        image = np.ndarray(shape=(height, width, 3), dtype=np.uint8, buffer=data.data)
    else:
        image = np.ndarray(shape=(height, width), dtype=np.uint8, buffer=data.data)


    # Undistort image
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
    dst = cv2.cvtColor(dst, cv2.COLOR_RGB2BGR)

    x, y, w, h = roi
    dst = dst[y:y+h, 113:526]
    print(w, h)

    cv2.imshow("Image window", dst)
    cv2.waitKey(3)
    
    global frame_num
    frame_num += 1
    if frame_num % 10 == 0:
        cv2.imwrite('/home/nvidia/xycar_ws/src/lane_detection_project/dataset/{}.jpg'.format(frame_num), dst)

def main():
    rospy.init_node('undistorted_image')
    image_sub = rospy.Subscriber("usb_cam/image_raw", Image, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
