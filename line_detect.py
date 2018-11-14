# -*- coding: utf-8 -*-
"""
Created on Wed Oct 24 16:44:10 2018

@author: igvc
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 14:43:46 2018
Hopefully detects white lines on a video of the IGVC course
@author:igvc/bill
"""

import cv2
import numpy as np
 
video = cv2.VideoCapture(-1)
 
while True:
    ret, orig_frame = video.read()
    if not ret:
        video = cv2.VideoCapture(-1)
        continue
 
    frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS) #Converts to frame to HSL
    low_white = np.array([0, 190, 0])       #These values are in HSL
    hi_white = np.array([180, 255, 255])        #same
    mask = cv2.inRange(hsv, low_white, hi_white)#maske gets anything in the range of those two values
    edges = cv2.Canny(mask, 75, 150)            #Performs a Canny edge detect on mask
 
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
 
    cv2.imshow("frame", frame)
    cv2.imshow("edges", mask)
 
    key = cv2.waitKey(25)
    if key == 27:
        break
video.release()
cv2.destroyAllWindows()
