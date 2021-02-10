#!/usr/bin/env python
from control import *
import rospy
import cv2 
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from matplotlib import pyplot as plt 
import numpy as np
import time
from std_msgs.msg import String
from math import degrees, atan
import sys

'''
def stop_callback(data):
    #print(data.data)
    if data.data == 'stop':
        change_stopped(True)
        stopping()
        #print("Stopped")
    elif data.data == 'go':
        change_stopped(False)
'''

def image_callback(image_message):
    #print("check");

    pub = rospy.Publisher("stop_send", String, queue_size=2)

    global confirmCount
    global terminated
    global stop_found
    
    # bridge = CvBridge();
    # frame = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough');
    np_arr = np.fromstring(image_message.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    x, y, w, h = 0, 0, 1000, 200
    cv2.rectangle(frame, (x, x), (x + w, y + h), (0,0,0), -1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
    blur1 = cv2.blur(gray,(5,5));
    th_img = cv2.adaptiveThreshold(blur1,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,33,10);
    blur = cv2.blur(th_img,(5,5));
    edge = cv2.Canny(blur, 180, 255, 30);

    inter_mask = inter_region(edge)

    lines = cv2.HoughLinesP(inter_mask, 1, np.pi/180, 30, None, minLineLength = 200, maxLineGap = 50);



    #time.sleep(0.1);

    
    '''
    lined_image = bridge.cv2_to_imgmsg(frame, encoding='rgb8')
    pub = rospy.Publisher("lined_image", Image, queue_size=1)
    pub.publish(lined_image)
    line_interpretations(lines.tolist())
    '''

    #polar_lines = convert_to_polar(lines);


    #Finding horizontal lines for intersection
    horizontal_lines = [];
    try:
        for line in lines:
            #If gradient within -0.05 to 0.05 then more or less horizontal
            x0 = line[0][0];
            y0 = line[0][1];
            x1 = line[0][2];
            y1 = line[0][3];
            if x1 - x0 == 0:
                gradient = 10000000    #tho this should be infinity
            else:
                gradient = (float(y1) - y0)/(x1 - x0)

            angle_gradient = degrees(atan(gradient))
            if (angle_gradient <= 10 and angle_gradient >= -10):
                horizontal_lines.append(line);
    except Exception:
        return
        

    '''
    if len(horizontal_lines) == 0:
        return;
    '''
    qualifiedCount = 0;
    for line in horizontal_lines:
        x0 = line[0][0];
        y0 = line[0][1];
        x1 = line[0][2];
        y1 = line[0][3];
        if (y0 > 450 and y1 > 450 and x0 < 350 and x1 > 290):
            qualifiedCount += 1;
            
    if (qualifiedCount > 1):
        print("OOOOOOOOOOOOOOOOOOOOOO")
        terminated = 1;



    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    stop_mask = stop_region(img_gray)

    stop_data = cv2.CascadeClassifier('/home/rsa/catkin_ws/src/comp3431-rsa/comp3431_starter/src/stop_data.xml') 
    

    # can change the values here of minSize so it detects faster/slower.  
    found = stop_data.detectMultiScale(stop_mask, minSize =(20, 20)) 
      
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
        stop_found = 1;		
        '''
        stop_str = "stop"   
        print("Stop found!")
        pub.publish(stop_str)
        '''
    # Creates the environment of  
    # the picture and shows it 
    else:
        stop_found = 0;
        '''
        stop_str = "go"
        pub.publish(stop_str)
        print("Go go")
        '''

    #determining what the robot should do
    if stop_found == 1:
        print("sign found!");
        if terminated == 1:
            stop_str = "stop"   
            print("intersection found!")
            time.sleep(3)
            print("stopped!")
            pub.publish(stop_str)
        else:
            stop_str = "go"
            pub.publish(stop_str)
            #print("Go go")
    else:
        stop_str = "go"
        pub.publish(stop_str)
        #print("Go go")
        terminated = 0;

    '''
    #draw stuff lines
    if horizontal_lines is not None:
        for i in range(len(horizontal_lines)):
            # print lines[i]
            points = horizontal_lines[i][0]
            starting = (points[0], points[1])
            ending = (points[2], points[3])
            cv2.line(frame, starting, ending, (0, 0, 255), 2)

    cv2.imshow('horizontal_lines',frame)
    time.sleep(0.001)
    cv2.waitKey(0)
    '''
    
    cv2.destroyAllWindows()
        

def inter_region(image):
    height, width = image.shape
    #print(height)
    #print(width)
    triangle = np.array([
                       [(0, height), (0, 8*height/10), (width/2, 5*height/11), (width, 8*height/10), (width, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, triangle, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def stop_region(image):
    height, width = image.shape
    #print(height)
    #print(width)
    # might want to tune the parameters below, those detail the corners of a square. Currently will not detect 
    # signs that are too non-centred.
    square = np.array([
                       [(200, height), (200, 0), (width-200, 0), (width-200, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, square, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask


def convert_to_polar(lines):
    polar_lines = []
    for i in range(len(lines)):
        # print "lines", lines[i][0]
        x0, y0, x1, y1 = lines[i][0]
        # print 'x0:', x0, 'y0:', y0, 'x1:', x1, 'y1:',y1
        if x1 - x0 == 0:
            gradient = 10000000    #tho this should be infinity
        else:
            gradient = (float(y1) - y0)/(x1 - x0)
            

        intercept = -gradient * x1 + y1
        polar_lines.append((gradient, intercept))
        # print "gradient:", gradient, 'intercept:', intercept

    return polar_lines

def stop_inter_detect():
    rospy.init_node('stop_inter_detect', anonymous = True)

    #self.intersection_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback);
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback, queue_size=1);
    # rospy.Subscriber("/raspicam_node/image", Image, image_callback)
    # rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    #rospy.Subscriber("intersection_send", String, queue_size = 1);
    
    #rospy.Subscriber("stop_send", String, stop_callback)
    
    rospy.spin()


if __name__ == '__main__':
    confirmCount = 0;
    stop_found = 0;
    terminated = 0;
    stop_inter_detect()