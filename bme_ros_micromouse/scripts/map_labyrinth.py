#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_odometry_data(msg):
    global roll, pitch, yaw
    global x, y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    check_orient()

def check_orient(threshold = 0.05):
    global orient
    # up
    if 1.57 - threshold < yaw and yaw < 1.57 + threshold:
        orient = 0
    # right
    elif 0 - threshold < yaw and yaw < 0 + threshold:
        orient = 1
    # down
    elif -1.57 - threshold < yaw and yaw < -1.57 + threshold:
        orient = 2
    # left
    elif 3.14 - threshold < yaw and yaw < -3.14 + threshold:
        orient = 3
    else:
        orient = -1

def get_lidar_data(msg):
    # range 0-719 -> -180° - +180°
    global back_dist, right_dist, front_dist, left_dist

    back_dist = msg.ranges[0]
    right_dist = msg.ranges[180]
    front_dist = msg.ranges[360]
    left_dist = msg.ranges[540]

    check_walls()

def check_walls():
    global back_wall, left_wall, front_wall, right_wall

    if back_dist < 0.2:
        back_wall = True
    else:
        back_wall = False

    if left_dist < 0.2:
        left_wall = True
    else:
        left_wall = False

    if front_dist < 0.2:
        front_wall = True
    else:
        front_wall = False

    if right_dist < 0.2:
        right_wall = True
    else:
        right_wall = False

    return back_wall, left_wall, front_wall, right_wall

def which_wall():
    if left_wall and not front_wall and not right_wall and not back_wall:
        if orient == 0:
            wall_type = 1
        elif orient == 1:
            wall_type = 2
        elif orient == 2:
            wall_type = 3
        elif orient == 3:
            wall_type = 4
    elif not left_wall and  front_wall and not right_wall and not back_wall:
        if orient == 0:
            wall_type = 2
        elif orient == 1:
            wall_type = 3
        elif orient == 2:
            wall_type = 4
        elif orient == 3:
            wall_type = 1
    elif not left_wall and not front_wall and right_wall and not back_wall:
        if orient == 0:
            wall_type = 3
        elif orient == 1:
            wall_type = 4
        elif orient == 2:
            wall_type = 1
        elif orient == 3:
            wall_type = 2
    elif not left_wall and not front_wall and not right_wall and  back_wall:
        if orient == 0:
            wall_type = 4
        elif orient == 1:
            wall_type = 1
        elif orient == 2:
            wall_type = 2
        elif orient == 3:
            wall_type = 3
    elif left_wall and not front_wall and not right_wall and back_wall:
        if orient == 0:
            wall_type = 5
        elif orient == 1:
            wall_type = 8
        elif orient == 2:
            wall_type = 7
        elif orient == 3:
            wall_type = 6
    elif not left_wall and not front_wall and right_wall and back_wall:
        if orient == 0:
            wall_type = 6
        elif orient == 1:
            wall_type = 5
        elif orient == 2:
            wall_type = 8
        elif orient == 3:
            wall_type = 7
    elif not left_wall and front_wall and right_wall and not back_wall:
        if orient == 0:
            wall_type = 7
        elif orient == 1:
            wall_type = 6
        elif orient == 2:
            wall_type = 5
        elif orient == 3:
            wall_type = 8
    elif left_wall and front_wall and not right_wall and not back_wall:
        if orient == 0:
            wall_type = 8
        elif orient == 1:
            wall_type = 7
        elif orient == 2:
            wall_type = 6
        elif orient == 3:
            wall_type = 5
    elif left_wall and not front_wall and right_wall and not back_wall:
        if orient == 0 or orient == 2:
            wall_type = 9
        elif orient == 1 or orient == 3:
            wall_type = 10
    elif not left_wall and front_wall and not right_wall and back_wall:
        if orient == 0 or orient == 2:
            wall_type = 10
        elif orient == 1 or orient == 3:
            wall_type = 9
    elif left_wall and not front_wall and right_wall and back_wall:
        if orient == 0:
            wall_type = 11
        elif orient == 1:
            wall_type = 14
        elif orient == 2:
            wall_type = 13
        elif orient == 3:
            wall_type = 12
    elif not left_wall and front_wall and right_wall and back_wall:
        if orient == 0:
            wall_type = 12
        elif orient == 1:
            wall_type = 11
        elif orient == 2:
            wall_type = 14
        elif orient == 3:
            wall_type = 13
    elif left_wall and front_wall and right_wall and not back_wall:
        if orient == 0:
            wall_type = 13
        elif orient == 1:
            wall_type = 12
        elif orient == 2:
            wall_type = 11
        elif orient == 3:
            wall_type = 14
    elif left_wall andfront_wall and not right_wall and back_wall:
        if orient == 0:
            wall_type = 14
        elif orient == 1:
            wall_type = 13
        elif orient == 2:
            wall_type = 12
        elif orient == 3:
            wall_type = 11
    elif not left_wall and not front_wall and not right_wall and not back_wall:
        if orient == 0 or orient == 1 or orient == 2 or orient == 3:
            wall_type = 15
    elif left_wall and front_wall and right_wall and back_wall:
        if orient == 0 or orient == 1 or orient == 2 or orient == 3:
            wall_type = 16

    return wall_type

def go_forward(publisher):
    curr_pos_x = x
    curr_pos_y = y
    move = Twist()
    print(orient)
    if orient == 0 or orient == 2:
        while abs(y - curr_pos_y) < (0.3 - DEACCELERATION_DIST):
            move.linear.x = 0.5
            publisher.publish(move)
    elif orient == 1 or orient == 3:
        while abs(x - curr_pos_x) < (0.3 - DEACCELERATION_DIST):
            move.linear.x = 0.5
            publisher.publish(move)

    move.linear.x = 0
    publisher.publish(move)
    rospy.sleep(3)

def turn(publisher, new_orient):
    curr_yaw = yaw
    move = Twist()
    while orient != new_orient:
        orient_diff = new_orient - orient
        print(orient)
        if orient == -1:
            orient_diff = prev_orient_diff
        if orient_diff == 1 or orient_diff == -3:
            move.angular.z = -0.3
            publisher.publish(move)
        else:
            move.angular.z = 0.3
            publisher.publish(move)
        prev_orient_diff = orient_diff
    move.angular.z = 0
    publisher.publish(move)
    rospy.sleep(3)

rospy.init_node('labyrinth_mapper')
rospy.loginfo("Labyrinth mapping started!")

sub_odom = rospy.Subscriber ('/odom', Odometry, get_odometry_data)
sub_lidar = rospy.Subscriber('/scan', LaserScan, get_lidar_data)
pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

DEACCELERATION_DIST = 0.18

roll, pitch, yaw = 0, 0, 1.57
x, y = 0, 0


back_wall, left_wall,front_wall, right_wall = True, True, False, True

cmd_vel = Twist()
cmd_vel.linear.x = 0
cmd_vel.angular.z = 0

orient = 0
maze_x, maze_y = 0, 10
# 7x11
maze = [[0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0]]

flood = [[6,5,4,3,2,1,0],
        [7,6,5,4,3,2,1],
        [8,7,6,5,4,3,2],
        [9,8,7,6,5,4,3],
        [10,9,8,7,6,5,4],
        [11,10,9,8,7,6,5],
        [12,11,10,9,8,7,6],
        [13,12,11,10,9,8,7],
        [14,13,12,11,10,9,8],
        [15,14,13,12,11,10,9],
        [16,15,14,13,12,11,10]]

flood2 = [[0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0]]

#while not rospy.is_shutdown():

go_forward(pub_vel)
turn(pub_vel, 1)
print(orient)