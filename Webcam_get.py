# -*- coding: utf-8 -*-
"""
Created on Tue Oct 30 14:20:13 2018

@author: igvc
"""

import pyrealsense2 as rs
#import numpy as np
#import cv2
#cap.open(0)
#cap = cv2.VideoCapture(0)
pipe = rs.pipeline()
profile = pipe.start()
for i in range(0, 100):
    frames = pipe.wait_for_frames()
    for f in frames:
      print(f.profile)
    pipe.stop()
#while(True):
    # Capture frame-by-frame
   #ret, frame = cap.read()

    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
 #   cv2.imshow('frame',frame)
  #  if cv2.waitKey(1) & 0xFF == ord('q'):
   #     break

# When everything done, release the capture
#cap.release()
#cv2.destroyAllWindows()