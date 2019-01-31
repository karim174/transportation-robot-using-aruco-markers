#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import cv2.aruco as aruco
import time
import serial
import high_level_three as cntrl

arduino = serial.Serial('/dev/ttyACM0',9600) # Establish the connection on a specific port
time.sleep(2) #wait for connection to be established

# DAMN_ITS_OFFSET
# calibration parameters from checkboard

camera_Matrix = np.array([[800.7466328584435, 0.0, 335.3685359582425],
                         [0.0, 804.1665602634919, 213.2964152084579],
                         [0.0, 0.0, 1.0]])

dist_Coff = np.array([0.5446652394728176, -5.5233881377989045,
                     -0.014441057933817907, 0.008332787614176447,
                     15.6171842384956675])

cap = cv2.VideoCapture(0)
cap.set(3, 640)  # set frame width
cap.set(4, 480)  # set frame height
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # disabling autofocus

# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

board = aruco.GridBoard_create(5, 7, 0.033, 0.006, dictionary)

arucoParams = aruco.DetectorParameters_create()  # detector with default parameter created
arucoParams.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
print(arucoParams.cornerRefinementMethod)
markerLength = 0.05  # side length of the printed tag is 0.05m.
ids = np.zeros((1,35),'uint8')
while True:

    # Capture frame-by-frame

    (ret, frame) = cap.read()

    # frame = frame[300:640,100:560]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('original', frame)

    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    cv2.imshow('grayed', gray)

    # ret3,gray = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # cv2.imshow("thresholded", gray)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=arucoParams)  # Detect aruco
    aruco.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints)
    if ids is not None:  # if the aruco marker detected

        # rvec, tvec, objpoints = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_Matrix, dist_Coff) # For a single marker
        # board = aruco.Board_create(objpoints,dictionary,ids)
        imgWithAruco = aruco.drawDetectedMarkers(frame, corners, ids,(0, 255, 0))
        retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_Matrix, dist_Coff)
        imgWithAruco = aruco.drawAxis(frame, camera_Matrix, dist_Coff, rvec, tvec, 0.1)
    # imgWithArucwo = aruco.drawAxis(imgWithAruco, camera_Matrix, dist_Coff, rvec, tvec, 10) #balash weghet nazar abo balash katar meno # axis length 100 can be changed according to your requirement

        x_dist = round(tvec[0][0] * 100, 1)
        y_dist = round(tvec[1][0] * 100, 1)
        z_dist = round(tvec[2][0] * 100, 1)
        x_rot = round(rvec[0][0], 2)
        y_rot = round(rvec[1][0], 2)
        z_rot = round(rvec[2][0], 2)
        pose = np.array([x_dist,z_dist,y_rot])
        speeds=cntrl.high_level_control('product',pose,0,offset=0,kp = 0.25,ka=0.3,kb=-0.005)
        print(speeds)
        wheels_s=cntrl.inverse(speeds,14.8,5)
        #print(wheels_s)
        wheels_spds_rounded = np.rint(wheels_s)
        wheels_spds_rint = wheels_spds_rounded.astype(int)
        arduino.write(bytes(chr(wheels_spds_rint[0]),"utf-8"))
        arduino.write(bytes(chr(wheels_spds_rint[1]),"utf-8"))
    else:
        arduino.write(bytes(chr(0),"utf-8"))
        arduino.write(bytes(chr(0),"utf-8"))

    cv2.imshow('withtag', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
