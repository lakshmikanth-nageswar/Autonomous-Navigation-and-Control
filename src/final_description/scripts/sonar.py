#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Range

def callback(msg):
    print(msg.range)

rospy.init_node('scan_values_1')
sub = rospy.Subscriber('/sensor/sonar_front', Range, callback)
rospy.spin()
