#!/usr/bin/env python

""" cv_bridge_demo.py - Version 1.1 2013-12-20

    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo():
    def __init__(self):
        self.node_name = "line_detect"
      
    	self.image_pub = rospy.Publisher("final_image",Image, queue_size=1)  
        


	rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        self.rgb_image = None
        #self.depth_image = None
        
        # Create the OpenCV display window for the RGB image
        self.rgb_window_name = self.node_name
        cv2.namedWindow(self.rgb_window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.rgb_window_name, 1280, 720)
                
        # And one for the depth image
        #self.depth_window_name = "Depth Image"

        #cv2.namedWindow(self.depth_window_name, cv2.WINDOW_NORMAL)
        #cv2.moveWindow(self.depth_window_name, 25, 350)
        
        # Create the cv_bridge object
        self.bridge = CvBridge() 
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        
        # Subscribe to the camera image and depth topics and set he appropriate callbacks
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        #self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=1)
        
        rospy.loginfo("Ready.")
        
        while not rospy.is_shutdown():
            if not self.rgb_image is None:
                cv2.imshow(self.rgb_window_name, self.rgb_image)
               
      
                # Process any keyboard commands
                self.keystroke = cv2.waitKey(5)
                         
                if self.keystroke != -1:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'q':
                        # The user has press the q key, so exit
                        rospy.signal_shutdown("User hit q key to quit.")
                
            #if not self.depth_image is None:
                #cv2.imshow(self.depth_window_name, self.depth_image)                

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Process the frame using the process_image() function
        self.rgb_image = self.process_image(frame)        
        
        try:
      	    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.rgb_image, "mono8"))
    	except CvBridgeError, e:
            print e
                
    #def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        #try:
            # Convert the depth image using the default passthrough encoding
            #depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
        #except CvBridgeError, e:
            #print e
                
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        #depth_image = cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32FC1)
        
        # Process the depth image
        #self.depth_image = self.process_depth_image(depth_image)

          
    def process_image(self, frame):
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS) #Converts to frame to HSL
	low_white = np.array([0, 190, 0])       #These values are in HSL
	hi_white = np.array([180, 255, 255])        #same
	mask = cv2.inRange(hsv, low_white, hi_white)#mask gets anything in the range of those two values
	edges = cv2.Canny(mask, 75, 150)            #Performs a Canny edge detect on mask
	pts1 = np.float32([[340, 225], [940, 225], [140, 600], [1140, 600]])   #Set ROI 
    	pts2 = np.float32([[0, 0], [1200, 0], [0, 700], [1200, 700]])    #Sets final pixel placement for ROI 
    	matrix = cv2.getPerspectiveTransform(pts1, pts2) 
	lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=5)     #performs HLT of edges
    
  
    	if lines is not None:
        	for line in lines:
            		x1, y1, x2, y2 = line[0]
            		cv2.line(mask, (x1, y1), (x2, y2), (0, 255, 0), 5)      #puts green lines over frame
            
    	result = cv2.warpPerspective(mask, matrix, (1280, 720))
        
        return result
    
    #def process_depth_image(self, frame):
        # Just return the raw image for this demo
        #return frame
    
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
