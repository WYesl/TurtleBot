#!/usr/bin/env python

import rospy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib import pyplot as plt 
import numpy as np
import time
from std_msgs.msg import String

def image_callback(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    
    pub = rospy.Publisher("stop_send", String, queue_size=10)
	  
	# OpenCV opens images as grayscale version 
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    mask = region(img_gray)
 
	# Use minSize because for not  
	# bothering with extra-small  
	# dots that would look like STOP signs 
    stop_data = cv2.CascadeClassifier('/home/rsa/catkin_ws/src/comp3431-rsa/comp3431_starter/src/stop_data.xml') 
    

	# can change the values here of minSize so it detects faster/slower.  
    found = stop_data.detectMultiScale(mask, minSize =(20, 20)) 
	  
	# Don't do anything if there's  
	# no sign 
    amount_found = len(found) 
	  
    if amount_found != 0: 
		  
		# There may be more than one 
		# sign in the image 
	
        for (x, y, width, height) in found: 
		      
		    # We draw a green rectangle around 
		    # every recognized sign 
            cv2.rectangle(frame, (x, y), (x + height, y + width), (0, 255, 0), 5) 
	
        stop_str = "stop"   
        print("Stop found!")
        pub.publish(stop_str)
	# Creates the environment of  
	# the picture and shows it 
    else:
        stop_str = "go"
        pub.publish(stop_str)
        print("Go go")
    #cv2.imshow("stop_found", frame)
    #time.sleep(0.005)
    #cv2.waitKey(0)

    cv2.destroyAllWindows()

def region(image):
    height, width = image.shape
    #print(height)
    #print(width)
    # might want to tune the parameters below, those detail the corners of a square. Currently will not detect 
    # signs that are too non-centred.
    size = 200
    square = np.array([
                       [(size, height), (size, 0), (width-size, 0), (width-size, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, square, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def stop_detect():
    rospy.init_node('stop_detect', anonymous=True)
    rospy.Subscriber("/raspicam_node/image/compressed", Image, image_callback)
    #rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    stop_detect()
