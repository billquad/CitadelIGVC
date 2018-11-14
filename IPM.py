# -*- coding: utf-8 -*-
"""
Created on Tue Oct 30 14:41:20 2018
Converts video stream to IPM (birds-eye)
Creates region of intererst (set by pts1)
@author: igvc/bill
"""

import cv2
import numpy as np
 
video = cv2.VideoCapture(-1)
 
while True:
    
    _, frame = video.read(-1) #Reads stream if (-1), stores as frame
    
    cv2.circle(frame, (60, 200), 5, (0, 0, 255), -1)
    cv2.circle(frame, (580, 200), 5, (0, 0, 255), -1)
    cv2.circle(frame, (5, 475), 5, (0, 0, 255), -1)
    cv2.circle(frame, (635, 475), 5, (0, 0, 255), -1)
 
    pts1 = np.float32([[60, 200], [580, 200], [5, 475], [635, 475]])
    pts2 = np.float32([[0, 0], [480, 0], [0, 640], [480, 640]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
 
    result = cv2.warpPerspective(frame, matrix, (480, 640))
 
 
    cv2.imshow("Frame", frame)
    cv2.imshow("Perspective transformation", result)
 
    key = cv2.waitKey(1)
    if key == 27:
        break
 
cap.release()
cv2.destroyAllWindows()