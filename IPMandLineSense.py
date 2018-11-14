


import cv2
import numpy as np
 
video = cv2.VideoCapture("IGVC_course.mp4")
 
while True:
    ret, orig_frame = video.read()
    if not ret:
        video = cv2.VideoCapture("IGVC_course.mp4")
        continue
 

    frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS) #Converts to frame to HSL
    low_white = np.array([0, 190, 0])       #These values are in HSL
    hi_white = np.array([180, 255, 255])        #same
    mask = cv2.inRange(hsv, low_white, hi_white)#maske gets anything in the range of those two values
    edges = cv2.Canny(mask, 75, 150)            #Performs a Canny edge detect on mask

    cv2.circle(frame, (340, 225), 5, (0, 0, 255), -1)
    cv2.circle(frame, (940, 225), 5, (0, 0, 255), -1)
    cv2.circle(frame, (140, 600), 5, (0, 0, 255), -1)
    cv2.circle(frame, (1140, 600), 5, (0, 0, 255), -1)    
    pts1 = np.float32([[340, 225], [940, 225], [140, 600], [1140, 600]])
    pts2 = np.float32([[0, 0], [1200, 0], [0, 700], [1200, 700]])    
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    
    
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
    
    result = cv2.warpPerspective(mask, matrix, (1200, 700))
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
    cv2.imshow("frame", frame)
    cv2.imshow("edges", mask)
    cv2.imshow("Perspective transformation", result)
    key = cv2.waitKey(25)
    if key == 27:
        break
video.release()
cv2.destroyAllWindows()