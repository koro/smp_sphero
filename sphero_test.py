#!/usr/bin/env python
"""test another command mode for sphero: linear velocity and heading"""

import rospy
import signal
import time, sys, argparse

from std_msgs.msg import Float32, Float32MultiArray, ColorRGBA
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion #, Point, Pose, TwistWithCovariance, Vector3
from smp_msgs.msg import reservoir
import tf

# pub_twist = rospy.Publisher("/cmd_vel_raw", Twist, queue_size=1)
pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
T = Twist()

rospy.init_node("sphero_test")

i = 0

while not rospy.is_shutdown():
    T.linear.x = 60
    # T.angular.z = i/10.
    T.linear.y = i/10.
    pub_twist.publish(T)
    i += 1
    time.sleep(0.1)
