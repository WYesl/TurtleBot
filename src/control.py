from geometry_msgs.msg import Twist
import rospy 
from math import atan, degrees, sqrt
from operator import itemgetter


stopped = False

def change_stopped(change):
    global stopped
    stopped = change

# Images are 480 x 640 pixels
# Left to right is 0 to 640 , top to down is 0 to 480
# Points are in the format of [x start, y start, x end, y end]
def line_interpretations(points):
    global stopped    
    print(stopped)

    # Don't do anything if robot is stopped
    if stopped == True:
        return
    
    relevant_lines = points

    # Average parameters of the points
    average_line = [] # The object containg the average line
    
    # The average line's properties
    average_angle = 0
    average_intercept = 0
    average_distance = 0
    for line in relevant_lines:
        average_angle += line[0][0]
        average_intercept += line[0][1]
        average_distance += line[1]
    average_angle = average_angle/len(relevant_lines)
    average_intercept /= len(relevant_lines)
    average_distance /= len(relevant_lines)

    average_line.append([(average_angle, average_intercept), average_distance])


    # line threshhold is where the robot should consider the line between turning left and right
    line_threshold = 165

    vel_msg = Twist()
    vel_msg.linear.y = vel_msg.linear.z = 0
    vel_msg.angular.y = vel_msg.angular.x = 0

    # parameters for the robot's turn and speed
    max_speed = 0.1
    max_turn = 0.2

    # default state is to make the robot go forward
    vel_msg.linear.x = max_speed
    vel_msg.angular.z = 0

    distance = average_distance

    # If no lines are found just move forward
    if len(relevant_lines) == 0:
        vel_msg.linear.x = max_speed * 0.05
        vel_msg.angular.z = 0

    elif distance > line_threshold:
        # turn robot to the left a little bit
        vel_msg.linear.x = max_speed * 0.3
        vel_msg.angular.z = max_turn * 0.2
        
    elif distance <= line_threshold - 10:
        if distance <= line_threshold - 50:
            vel_msg.linear.x = max_speed * 0.1
            vel_msg.angular.z = -max_turn * 0.4
        else:      
            # turn robot to the right a little bit
            vel_msg.linear.x = max_speed * 0.3
            vel_msg.angular.z = -max_turn * 0.2
    
    send_message(vel_msg)

def line_interpretations_gradient(points):
    polar_lines = convert_to_polar(points)
    reference0 = (280, 400)
    relevant_lines0 = find_closest_left_lines(polar_lines, reference0, 2, False)

    vel_msg = Twist()
    vel_msg.linear.y = vel_msg.linear.z = 0
    vel_msg.angular.y = vel_msg.angular.x = 0
    # parameters for the robot's turn and speed
    max_speed = 0.1
    max_turn = 0.1


    # If relevant lines are seen
    if len(relevant_lines0) == 0:
        print("lines returned 0")
        # Keep moving forward
        vel_msg.linear.x = max_speed * 0.05
        vel_msg.angular.z = 0
        send_message(vel_msg)
    else:
        print("lines didn't return 0")
        # this below basically computes the average angle of the lines returned 
        # in degrees 
        print(relevant_lines0)

        sum0 = 0
        sum_intercept0 = 0
        for line in relevant_lines0:
            sum0 += degrees(atan(line[0][0]))
            sum_intercept0 += line[0][1]
        
        average_intercept0 = sum_intercept0/len(relevant_lines0)
        average_angle0 = abs(sum0 / len(relevant_lines0))


        print("average_angle0:", average_angle0)
        
        # Legacy purposes
        ave_combined_angle =  average_angle0
        print("avecombined:", ave_combined_angle)
        
        # Around 30 degrees is what we would consider to be driving straight
        # The below variables is for the equation y=1/(x-a) + b
        print("average_intercept0:", average_intercept0)
        if average_intercept0 >= 480:
            vel_msg.linear.x = 0
            vel_msg.angular.z = max_turn * 0.1
        elif average_intercept0 <= 400:
            vel_msg.linear.x = 0
            vel_msg.angular.z = max_turn * 0.1
        else:
            straight_angle = 32
            a = 0.5 - sqrt(3)/2
            b = 2 + 2/(1-sqrt(3))
            if ave_combined_angle <= straight_angle * 2:
                if ave_combined_angle <= straight_angle:
                    vel_msg.linear.x = max_speed * (1 - abs(1-ave_combined_angle/straight_angle)) * 0.3
                    vel_msg.angular.z = max_turn * -1 * (1/(ave_combined_angle/straight_angle - a) + b) * 0.5
                else:
                    vel_msg.linear.x = max_speed * abs(ave_combined_angle/straight_angle) * 0.3
                    vel_msg.angular.z = max_turn * (1/ (-ave_combined_angle/straight_angle - a) + b) * 0.5 
        
        send_message(vel_msg)

    
# Takes in a Twist object 
def send_message(message):
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    pub.publish(message)
    print("published")



# not really polar coordinates its actually just hough interpretation
# polar format in this case is just (gradient, intercept)
def convert_to_polar(lines):
    polar_lines = []
    for i in range(len(lines)):
        x0, y0, x1, y1 = lines[i][0]
        if x1 - x0 == 0:
            continue
        else:
            gradient = (float(y1) - y0)/(x1 - x0)
            # remove high gradient lines
            # this might affect left turns tho so might need to change this
            if degrees(atan(abs(gradient))) >= 75:
                continue
            

        intercept = -gradient * x1 + y1
        polar_lines.append((gradient, intercept))
        # print "gradient:", gradient, 'intercept:', intercept

    return polar_lines
        
# finds the closest lines on the left 
# lines is just the data 
# num is the number of lines you want to return 

# returns a list of lists with the following format
# [[line definition, distance to line], [line definition, distance to line], ...]
# if flag is true then remove all x_values less than 0 else it doesnt 
def find_closest_left_lines(lines, reference, num, flag):
    # reference is the x y coordinates of the thing
    relevant_lines = []
    for line in lines:
        # if gradient of the line is 0 then continue 
        if line[0] == 0:
            continue
        
        x_value = (reference[1] - line[1])/line[0]
        if x_value < 0 and flag == True:
            continue

        distance = reference[0] - x_value
        # If distance is less than 0 then the line is on the right hand side therefore not needed
        # We only accept lines on the left hand side
        if distance > 0:
            relevant_lines.append([line, distance])

    relevant_lines = sorted(relevant_lines, key=itemgetter(1))
    return relevant_lines[:num]

# Stops the robot and publishes it to the cmd_vel topic
def stopping():
	vel_msg = Twist()
	vel_msg.linear.y = vel_msg.linear.z = vel_msg.linear.x = 0
	vel_msg.angular.y = vel_msg.angular.x =  0
	vel_msg.angular.z = 0
	pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
	pub.publish(vel_msg)
