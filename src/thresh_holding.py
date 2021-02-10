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
    #print(data.data)
    if data.data == 'stop':
        change_stopped(True)
        stopping()
        #print("Stopped")
    elif data.data == 'go':
        change_stopped(False)
        #print("Go")
    #else:
     #   print("?????")

def image_callback(data):
    # bridge = CvBridge()
    # frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    np_arr = np.fromstring(data.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # frame = bridge.imgmsg_to_cv2(data)
    # ret, frame = cv_image.read()
    # if ret:
    x, y, w, h = 0, 0, 1000, 200
    cv2.rectangle(frame, (x, x), (x + w, y + h), (0,0,0), -1)
    print "new frame"
    # cv2.imshow("hello", frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur1 = cv2.blur(gray,(5,5))
    # _, th_img = cv2.threshold(blur1,170,255,cv2.THRESH_BINARY)
    th_img = cv2.adaptiveThreshold(blur1,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,33,10)
    blur = cv2.blur(th_img,(5,5))

    edge = cv2.Canny(blur, 180, 255, 30)
    # lines = cv2.HoughLines(edge,1,np.pi/180, 200,20,0)

    # mask = region(edge)
    print edge.shape

    lines = cv2.HoughLinesP(edge, 1, np.pi/180, 30, None, minLineLength = 5, maxLineGap = 15)

    # this below part is used for the HoughLines 
    # if lines is not None:
    #     for i in range(len(lines)):
    #         rho, theta = lines[i][0]
    #         a = np.cos(theta)
    #         b = np.sin(theta)
    #         x0 = a*rho; y0 = b*rho
            
    #         xs = [int(x0 + 1000*(-b)), int(x0 - 1000*(-b))]
    #         ys = [int(y0 + 1000*(a)), int(y0 - 1000*(a))]
            # cv2.line(frame,(int(x0 + 1000*(-b)), int(y0 + 1000*(a))),( int(x0 - 1000*(-b)),int(y0 - 1000*(a)) ), (255, 0, 0), 2)


    # preemptively find the relevent lines 
    
    polar = convert_to_polar(lines.tolist())
    reference0 = (280, 400)
    relevent = find_closest_left_lines(polar, reference0, 3, False)
    print "relevant_lines in thresh:", relevent

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

    # this part is used for the HoughLinesP
    # if lines is not None:e
        
    # 	for i in range(len(lines)):
    # 		# print lines[i]
    # 		points = lines[i][0]
    # 		starting = (points[0], points[1])
    # 		ending = (points[2], points[3])
    # 		cv2.line(frame, starting, ending, (255, 0, 0), 2)
        
    # this part is used for relevent lines
    for i in range(len(relevent_lines)):
            # print lines[i]
            points = relevent_lines[i]
            starting = (int(points[0]), int(points[1]))
            ending = (int(points[2]), int(points[3]))
            cv2.line(frame, starting, ending, (255, 0, 0), 2)

    # lines_image = bridge.cv2_to_imgmsg(lines, encoding="passthrough")
    # lines_image_pub = rospy.Publisher("COOL_IMAGE_LINES", Image, queue_size=10)
    # lines_image_pub.publish(lines_image)
    # cv2.imshow('frame',gray)
    # cv2.imshow('thresholded',th_img)
    # cv2.imshow('edge',edge)
    
    # cv2.imshow('lines',frame)
    time.sleep(0.01)
    # cv2.waitKey(0)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    # ros

    # cap.release()
    # frame.release()
    cv2.destroyAllWindows()
    # lined_image = bridge.cv2_to_imgmsg(frame, encoding='rgb8')
    # pub = rospy.Publisher("lined_image", Image, queue_size=1)
    # pub.publish(lined_image)
    # line_interpretations(lines.tolist())
    # line_interpretations_gradient(lines.tolist())
    line_interpretations(relevent)
    print "end"

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

def threshholding():
    rospy.init_node('threshholding', anonymous=True)
    # while not rospy.core.is_shutdown():
    # rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback)
    # rospy.Subscriber("/raspicam_node/image", Image, image_callback)
    rospy.Subscriber("stop_send", String, stop_callback)
    rospy.spin()
    print "stopping program"
    if rospy.is_shutdown():
        time.sleep(1)
        vel_msg = Twist()
        vel_msg.linear.y = vel_msg.linear.z = vel_msg.linear.x = 0
        vel_msg.angular.y = vel_msg.angular.x = vel_msg.angular.z = 0
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        pub.publish(vel_msg)
        print 'sent message'

if __name__ == '__main__':
    print "starting"
    threshholding()	
