#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time


rospy.init_node('ball_dumping', anonymous=True)
clearance = Float32()
clearance_change = Float32()

## Flaps and Basic Functions ##
pub_fg = rospy.Publisher('/bot1_frontgate_controller/command', Float64, queue_size=10)
pub_bg = rospy.Publisher('/bot1_backgate_controller/command', Float64, queue_size=10)
pub_df = rospy.Publisher('/bot1_diffdrive_controller/cmd_vel', Twist, queue_size=10)

def callback(msg):
	global clearance
	clearance = msg.range
	print(clearance)


def clearance_change():
	global clearance_change
	
	old_dist = clearance
	time.sleep(0.5)
	new_dist = clearance
	
	clearance_change = new_dist-old_dist

def traverse(v,t):
	speed = Twist()
	speed.linear.x = v
	speed.angular.z = 0
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate = rospy.Rate(10)
	while (t1 - t0) < t:
		pub_df.publish(speed)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()


def bg_close():
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate2 = rospy.Rate(25)
	while (t1 - t0) < 2:
		pub_bg.publish(2)
		t1 = rospy.Time.now().to_sec()
		rate2.sleep()


def bg_open():
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate = rospy.Rate(10)
	while (t1 - t0) < 4:
		pub_bg.publish(-10)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()


def rotate(value, t):
	angle = Twist()
	angle.linear.x = 0
	angle.angular.z = value
	t0 = rospy.Time.now().to_sec()
	t1 = t0
	rate = rospy.Rate(10)
	while (t1 - t0) < t:
		pub_df.publish(angle)
		t1 = rospy.Time.now().to_sec()
		rate.sleep()


def dump():
	
	bg_close()
	
	rotate(0.5,10)
	
	bg_open()



#### RUN ####

sub_sn = rospy.Subscriber('/sensor/sonar_front', Range, callback)

print(1)
rate = rospy.Rate(10)

print(clearance)

while clearance < 0.37:
	print(clearance)
	
	speed = Twist()
	speed.linear.x = 0.15
	speed.angular.z = 0
	pub_df.publish(speed)
	 
	rate.sleep()
	

stop = Twist()
stop.linear.x = 0
stop.angular.z = 0
pub_df.publish(stop)





