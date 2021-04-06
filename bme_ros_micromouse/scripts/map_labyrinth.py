#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation(msg):
    global roll, pitch, yaw
    global x, y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    print(y)

    check_orient()


def check_orient(threshold = 0.2):
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

    #TODO if orient == -1


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
    
    debug = False
    if debug:
        print("---")
        print("back:" + str(back_wall))
        print("right:" + str(right_wall))
        print("front:" + str(front_wall))
        print("left:" + str(left_wall))
        print("---")
        print("back:" + str(back_dist))
        print("right:" + str(right_dist))
        print("front:" + str(front_dist))
        print("left:" + str(left_dist))

    return back_wall, left_wall, front_wall, right_wall

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
    while abs(yaw - curr_yaw) < 1.57:
        print(orient)
        move.angular.z = -0.1
        publisher.publish(move)
    move.angular.z = 0
    publisher.publish(move)
    rospy.sleep(3)

rospy.init_node('labyrinth_mapper')
rospy.loginfo("Labyrinth mapping started!")

sub_odom = rospy.Subscriber ('/odom', Odometry, get_rotation)
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
go_forward(pub_vel)
go_forward(pub_vel)
go_forward(pub_vel)
go_forward(pub_vel)