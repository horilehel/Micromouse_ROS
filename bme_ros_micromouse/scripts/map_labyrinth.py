#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

rospy.init_node('labyrinth_mapper')
rospy.loginfo("Labyrinth mapping started!")

sub_odom = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

roll, pitch, yaw = 0, 0, 0

cmd_vel = Twist()
cmd_vel.linear.x = 0
cmd_vel.angular.z = 0

while not rospy.is_shutdown():
    cmd_vel.linear.x = 0.5
    pub.publish(cmd_vel)