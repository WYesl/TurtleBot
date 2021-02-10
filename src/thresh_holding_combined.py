#!/usr/bin/env python
from control import *
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import time
from std_msgs.msg import String

# cap = cv2.VideoCapture('vid.mp4')

def stop_callback(data):
    if data == 'stop':
        change_stopped(True)
        stopping()
    elif data == 'go':
        change_stopped(False)
        

def image_callback(data):
    global confirmCount
    global terminated
    global stop_found

    # Find Lines
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    x, y, w, h = 0, 0, 1000, 200
    cv2.rectangle(frame, (x, x), (x + w, y + h), (0,0,0), -1)
    print("new frame")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur1 = cv2.blur(gray,(5,5))
    th_img = cv2.adaptiveThreshold(blur1,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,33,10)
    blur = cv2.blur(th_img,(5,5))

    edge = cv2.Canny(blur, 180, 255, 30)
    line_mask = line_region(edge)

    lines_lane = cv2.HoughLinesP(line_mask, 1, np.pi/180, 30, None, minLineLength = 5, maxLineGap = 15)


    # preemptively find the relevent lines 
    polar = convert_to_polar(lines_lane.tolist())
    reference0 = (280, 400)
    relevent = find_closest_left_lines(polar, reference0, 3, False)

    # relevant lines is for printing while relevent is for passing to the control functions
    relevent_lines = []
    for line in relevent:
        # y = mx + b
        x_start  = 0
        y_start = line[0][1]
        x_end = -line[0][1]/line[0][0]
        y_end = 0
        l = [x_start, y_start, x_end, y_end]
        relevent_lines.append(l)
        
    # This part is used for relevent lines
    for i in range(len(relevent_lines)):
            points = relevent_lines[i]
            starting = (int(points[0]), int(points[1]))
            ending = (int(points[2]), int(points[3]))
            cv2.line(frame, starting, ending, (255, 0, 0), 2)

    
    cv2.imshow('edge', edge)
    
    time.sleep(0.01)
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()

    inter_mask = inter_region(edge)

    lines_inter = cv2.HoughLinesP(inter_mask, 1, np.pi/180, 30, None, minLineLength = 200, maxLineGap = 50)
    
    # Finding horizontal lines for intersection
    horizontal_lines = []
    try:
        for line in lines_inter:
            
            # If gradient within -0.05 to 0.05 then more or less horizontal
            x0 = line[0][0]
            y0 = line[0][1]
            x1 = line[0][2]
            y1 = line[0][3]
            if x1 - x0 == 0:
                # Placeholder for infinity
                gradient = 10000000
            else:
                gradient = (float(y1) - y0)/(x1 - x0)

            angle_gradient = degrees(atan(gradient))
            if (angle_gradient <= 10 and angle_gradient >= -10):
                horizontal_lines.append(line)

    except Exception:
        pass
     
    qualifiedCount = 0
    for line in horizontal_lines:
        x0 = line[0][0]
        y0 = line[0][1]
        x1 = line[0][2]
        y1 = line[0][3]
        if (y0 > 450 and y1 > 450 and x0 < 350 and x1 > 290):
            qualifiedCount += 1
            
    if (qualifiedCount > 1):
        print("OOOOOOOOOOOOOOOOOOOOOO")
        # Terminated means there is an intersection
        terminated = 1
    

    # Stop sign detection
    img_gray_stop_detection = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    stop_mask = stop_region(img_gray_stop_detection)

    stop_data = cv2.CascadeClassifier('/home/rsa/catkin_ws/src/comp3431-rsa/comp3431_starter/src/stop_data.xml') 
    # can change the values here of minSize so it detects faster/slower.  
    found = stop_data.detectMultiScale(stop_mask, minSize =(20, 20)) 
     
    amount_found = len(found) 
    # If sign found
    if amount_found != 0: 
        
        # There may be more than one sign in the image 
        for (x, y, width, height) in found: 
            
            # We draw a green rectangle around every recognized sign for lined_image
            cv2.rectangle(frame, (x, y), (x + height, y + width), (0, 255, 0), 5) 
        stop_found = 1	

    # Else, set variable to 0
    else:
        stop_found = 0

    # Determining what the robot should do

    # If there is a sign and intersection, then stop
    # Else, go
    if stop_found == 1:
        print("sign found!")
        if terminated == 1:
            stop_str = "stop"   
            print("intersection found!")
            print("stopped!")
            stop_callback(stop_str)
        else:
            stop_str = "go"
            stop_callback(stop_str)

    else:
        stop_str = "go"
        stop_callback(stop_str)
        terminated = 0
    
    print("end")
    line_interpretations(relevent)

def region(image):
    height, width = image.shape
    # print(height)
    # print(width)
    triangle = np.array([
                       [(0, height), (0, 8*height/10), (width/2, 5*height/11), (width, 8*height/10), (width, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, triangle, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def stop_region(image):
    height, width = image.shape

    # Currently will not detect signs that are too non-centred, as parameters are for corners of a square
    square = np.array([
                       [(150, height), (150, 0), (width-150, 0), (width-150, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, square, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def line_region(image):
    height, width = image.shape
    rectangle = np.array([
                       [(0, height), (0, 2*height/10), (7*width/10, 2*height/10), (7*width/10, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, rectangle, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def inter_region(image):
    height, width = image.shape
    triangle = np.array([
                       [(0, height), (0, 8*height/10), (width/2, 5*height/11), (width, 8*height/10), (width, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, triangle, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask


def threshholding():
    rospy.init_node('threshholding', anonymous=True)
    rospy.Subscriber("/raspicam_node/image", Image, image_callback)
    
    rospy.spin()
    print("stopping program")
    if rospy.is_shutdown():
        time.sleep(1)
        vel_msg = Twist()
        vel_msg.linear.y = vel_msg.linear.z = vel_msg.linear.x = 0
        vel_msg.angular.y = vel_msg.angular.x = vel_msg.angular.z = 0
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        pub.publish(vel_msg)
        print('sent message')

if __name__ == '__main__':
    confirmCount = 0
    stop_found = 0
    terminated = 0
    print("starting")
    threshholding()	
