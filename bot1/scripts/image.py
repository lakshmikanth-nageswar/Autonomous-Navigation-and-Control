#!/usr/bin/env python
 

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
#problem1 - bot may turn first as cx,cy might go down before laser reads 22
#problem2- not following ball

import rospy
import numpy as np
from math import atan2,asin
from std_msgs.msg import String, Float32, MultiArrayDimension, Float64
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, LaserScan
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import math
import time
 

bridge = CvBridge()
pub = rospy.Publisher('/bot1_diffdrive_controller/cmd_vel', Twist, queue_size=10)

msg=Twist()

## Flaps and Basic Functions ##
pub_fg = rospy.Publisher('/bot1_frontgate_controller/command', Float64, queue_size=10)
pub_lf = rospy.Publisher('/bot1_lf_controller/command', Float64, queue_size=10)
pub_rf = rospy.Publisher('/bot1_rf_controller/command', Float64, queue_size=10)
pub_bg = rospy.Publisher('/bot1_backgate_controller/command', Float64, queue_size=10)
#pub_df = rospy.Publisher('/bot1_diffdrive_controller/cmd_vel', Twist, queue_size=10)

pi = math.pi


def fg_close(t):
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate2 = rospy.Rate(20)
	while (t1 - t0) < t:
		pub_fg.publish(20)
		t1 = rospy.Time.now().to_sec()
		rate2.sleep()

def fg_open():
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate = rospy.Rate(20)
	while (t1 - t0) < 4:
		pub_fg.publish(-5)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()

def init_flaps_open():
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate = rospy.Rate(10)
	while (t1 - t0) < 2:
		pub_lf.publish(1)
		pub_rf.publish(-1)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()


def flaps_open(x):
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate = rospy.Rate(20)
	while (t1 - t0) < 3:
		pub_lf.publish(x)
		pub_rf.publish(-x)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()

def flaps_close(x):
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate3 = rospy.Rate(20)
	while (t1 - t0) < 2:
		pub_lf.publish(-x)
		pub_rf.publish(x)
		t1 = rospy.Time.now().to_sec()
		rate3.sleep()

def rotate(value, t):
	angle = Twist()
	angle.linear.x = 0
	angle.angular.z = value
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate = rospy.Rate(10)
	while (t1 - t0) < t:
		pub.publish(angle)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()

def bg_close():
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate2 = rospy.Rate(50)
	while (t1 - t0) < 3:
		pub_bg.publish(-5)
		t1 = rospy.Time.now().to_sec()
		rate2.sleep()

def bg_open():
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	while (t1 - t0) < 3:
		pub_bg.publish(-10)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()

def rotate(value, t):
	angle = Twist()
	angle.linear.x = 0
	angle.angular.z = value

	rate = rospy.Rate(20)

	t0 = rospy.Time.now().to_sec()
	t1 = t0
	while (t1 - t0) < t:
		pub.publish(angle)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()

def traverse(v,t):
	speed = Twist()
	speed.linear.x = v
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate_df = rospy.Rate(50)
	while (t1 - t0) < t:
		pub.publish(speed)
		t1 = rospy.Time.now().to_sec()
		rate_df.sleep()

	speed.linear.x = 0
	pub_df.publish(speed)


### ball_control DEFINITION ###

def ball_control(color):
	 
	
	if color == "red":
		fg_close(2)
		flaps_close(1)
		
		rotate(0.3, 2)
		time.sleep(0.5)
		flaps_open(30)
		
		rotate(-0.3, 2.5)

		
	elif color == "green":
		go_forward=Twist()
		fg_close(2)

		go_forward.linear.x=0.192
		pub.publish(go_forward)
		fg_open()
		flaps_close(40)
		time.sleep(2)
		fg_close()
		flaps_open(1)


### DUMPING THE BALLS ###

def dump():

	rate = rospy.Rate(30)	
	
	traverse(-0.3, 6)
	rotate(pi/4, 11)
	print(1)
	bg_open()
	traverse(-0.8, 2)
	time.sleep(5)

def calculate_lines(frame, lines):
    # Empty arrays to store the coordinates of the left and right lines
    left = []
    right = []
    # Loops through every detected line
    for line in lines:
        # Reshapes line from 2D array to 1D array
        x1, y1, x2, y2 = line.reshape(4)
        # Fits a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
        # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
        #if slope < 0:
           # left.append((slope, y_intercept))
        #else:
            #right.append((slope, y_intercept))
        left.append((slope,y_intercept))
    # Averages out all the values for left and right into a single slope and y-intercept value for each line
    left_avg = np.average(left, axis = 0)
    #right_avg = np.average(right, axis = 0)
    # Calculates the x1, y1, x2, y2 coordinates for the left and right lines
    left_line = calculate_coordinates(frame, left_avg)
    #right_line = calculate_coordinates(frame, right_avg)
    return np.array([left_line])

def calculate_coordinates(frame, parameters):
    slope, intercept = parameters
    # Sets initial y-coordinate as height from top down (bottom of the frame)
    y1 = frame.shape[0]
    # Sets final y-coordinate as 150 above the bottom of the frame
    y2 = int(y1 -200)#- 150)
    # Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b
    x1 = int((y1 - intercept) / slope)
    # Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])
def visualize_lines(frame, lines):
    # Creates an image filled with zero intensities with the same dimensions as the frame
    lines_visualize = np.zeros_like(frame)
    # Checks if any lines are detected
    if lines is not None:
        #m=np.zeros(4)
        for x1, y1, x2, y2 in lines:
            #m=[x1,y1,x2,y2]+m
            # Draws lines between two coordinates with green color and 5 thickness
            cv2.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 5)
            
        #m=m/2
        #cv2.line(lines_visualize, (int(m[0]), int(m[1])), (int(m[2]), int(m[3])), (0, 255, 255), 5)

    return lines_visualize

def show_image(img):
    #cv2.imwrite('/home/aryan/mybot_ws/src/mybot_description/scripts/lol.jpg',img)
    #img=cv2.imread('/home/aryan/mybot_ws/src/mybot_description/scripts/lol.jpg')
    cv2.imshow('',img)
    cv2.waitKey(3)

def controls(lx,ly): 
	error=lx-200
	msg.angular.z=-(error/1000) 
	msg.linear.x=0.1-abs(error/2000)
	pub.publish(msg)

 
##########################################
direction='none'
ball_color='None'
bx=200
by=400
def arrow_placeholder():
	rospy.Subscriber("/arrow_msg", String, arrow_callback)

def laser_placeholder():
	rospy.Subscriber('/umic_bot/laser/scan', LaserScan, laser_callback)

def centroid_placeholder():
	rospy.Subscriber('/ball_msg', MultiArrayDimension, ball_callback)

def ball_callback(centroid_msg):
	global ball_color
	global bx
	global by
	ball_color=centroid_msg.label
	bx=centroid_msg.size
	by=centroid_msg.stride

def laser_callback(laser_msg):
	global laser_distance
	laser_distance=min(laser_msg.ranges)


def arrow_callback(str_msg):
	if str_msg=='right'or'left':
		global direction
		direction=str_msg
	 
	#rospy.loginfo(direction)
#########################		
def ball_follow_control(bx):
	error=bx-200
	msg.angular.z=-(error/100) 
	msg.linear.x=0.1-abs(error/2000)
	pub.publish(msg)
	print('lol')


############################
def image_callback(img_msg):
	
    #print(direction) 
    laser_placeholder()
    rospy.loginfo(img_msg.header)
    print('the laser distance is: {}'.format(laser_distance))
    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #show_image(cv_image)
        cv2.imwrite('/home/gangadhar/project_umic/src/bot1/scripts/lol.jpg',cv_image)
        img=cv2.imread('/home/gangadhar/project_umic/src/bot1/scripts/lol.jpg')
        img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (400, 400))  
        color_boundaries = [ # colors can be changed
	#([17, 15, 100], [90, 96, 255]), #red
	#([20, 90, 4], [90, 255, 100]), #green
	    ([50, 86, 86], [76, 140, 140]) ]# yellow]
        
        for (lower,upper) in color_boundaries:

            lower=np.array(lower,dtype='uint8')
            upper=np.array(upper,dtype='uint8')
            mask=cv2.inRange(img,lower,upper)
            output=cv2.bitwise_and(img,img,mask=mask)

        segment=output    
        m=cv2.moments(mask[0:300,:],False)
        try:
            cx,cy=m['m10']/m['m00'],m['m01']/m['m00']
        except ZeroDivisionError:
            cx,cy=200,400
        centroid = cv2.circle(segment, (int(cx),int(cy)), radius=5, color=(15, 150, 255), thickness=-1)
        
        r=cv2.moments(mask[250:,250:],False)
        try:
            rx,ry=r['m10']/r['m00'],r['m01']/r['m00']
        except ZeroDivisionError:
            rx,ry=75,75
        centroid_r = cv2.circle(segment, (int(rx)+250,int(ry)+250), radius=5, color=(255, 0, 0), thickness=-1) 
        
        #segment=output #+masks[1]+masks[2]
       # gray=cv2.cvtColor(segment,cv2.COLOR_BGR2GRAY)
       # kernel = np.ones((7,7), np.uint)
       # gray=cv2.dilate(gray, kernel, iterations=1) 
      #  ret,thresh1 = cv2.threshold(gray,10,255,cv2.THRESH_BINARY)
      #  lane = cv2.Canny(thresh1,90,170)
      #  hough = cv2.HoughLinesP(lane, 1, np.pi / 180, 80, np.array([]), minLineLength = 5, maxLineGap = 50)
        #try:
             
         #   lines = calculate_lines(img, hough)
        #lines_visualize = visualize_lines(img, lines)
         #   cv2.line(segment, (lines[0][0], lines[0][1]), (lines[0][2], lines[0][3]), (0, 255, 0), 3)
         #   cv2.imshow('',segment) 
        #except:
           # cv2.imshow('',segment)
        

        centroid_placeholder()
        
        print('ball color is: {} , bx = {} , by = {}'.format(ball_color,bx,by))

        arrow_placeholder()

        #arrow_work(cx,cy,centroid)
         
        if laser_distance <0.24:
        	msg.linear.x=0
        	msg.angular.z=0
        	pub.publish(msg)
        	time.sleep(0.1)
        	if ball_color=='red':
        		ball_control('red')
        	elif ball_color=='green':
        		ball_control('green')

        #if ball_color!='None':
        #	controls(bx,by)
        #elif ball_color!='None':
        #	ball_follow_control(bx)
        elif cx==200 and cy==400: #and direction=='right':

            msg.linear.x=0
            msg.angular.z=0
            pub.publish(msg)
            time.sleep(0.1)
            print(direction)
            msg.linear.x=0
            if direction.data=='right':
                msg.angular.z=-0.1
                
            if direction.data=='left':
                msg.angular.z=0.1
                 
            #else:
    	     #   msg.angular.z=-100
             #time.sleep(0.2)
            pub.publish(msg)
            time.sleep(0.1)
 
        else:
            controls(cx,cy)
        #if ry>76 and cx==200 and cy==400:
        	#turn_controls(rx,ry)

        show_image(centroid)

        #cv2.imshow('',centroid)
        #cv2.waitKey(100)
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def listener():
    
    rospy.init_node('line_follower', anonymous=False)
    rotate(0,1)
    init_flaps_open()
    
    rospy.Subscriber("/mybot/camera1/image_raw", Image, image_callback)
    #rospy.Subscriber("/arrow_msg", String, arrow_callback)
    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
 
 
if __name__ == '__main__':
	listener()
    #rospy.Subscriber("/arrow_msg", String, arrow_callback)
    #rospy.spin()
 
#cv2.namedWindow("Image Window", 1)

 
 # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
#while not rospy.is_shutdown():
    
  #  rospy.spin()
