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

def check_orient(threshold = 0.04):
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
    elif (3.14 - threshold < yaw and yaw < 3.14156) or (-3.14156 < yaw and yaw < -3.14 + threshold):
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

    if back_dist < 0.25:
        back_wall = True
    else:
        back_wall = False

    if left_dist < 0.25:
        left_wall = True
    else:
        left_wall = False

    if front_dist < 0.25:
        front_wall = True
    else:
        front_wall = False

    if right_dist < 0.25:
        right_wall = True
    else:
        right_wall = False

    return back_wall, left_wall, front_wall, right_wall

def which_wall():
    wall_type = 0
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
    elif left_wall and front_wall and not right_wall and back_wall:
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

def has_wall(wall_type, orient):
    front_wall_types = [2,7,8,10,12,13,14,16]
    right_wall_types = [3,6,7,9,11,12,13,16]
    back_wall_types = [4,5,6,10,11,12,14,16]
    left_wall_types = [1,5,8,9,11,13,14,16]

    wall = False

    if orient == 0:
        if wall_type in front_wall_types:
            wall = True
        else:
            wall = False
    elif orient == 1:
        if wall_type in right_wall_types:
            wall = True
        else:
            wall = False
    elif orient == 2:
        if wall_type in back_wall_types:
            wall = True
        else:
            wall = False
    elif orient == 3:
        if wall_type in left_wall_types:
            wall = True
        else:
            wall = False

    return wall

def go_forward(publisher):
    curr_pos_x = x
    curr_pos_y = y
    move = Twist()
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
    prev_orient_diff = 0
    while orient != new_orient:
        orient_diff = new_orient - orient
        if orient == -1:
            orient_diff = prev_orient_diff
        if orient_diff == 1 or orient_diff == -3:
            move.angular.z = -0.25
            publisher.publish(move)
        else:
            move.angular.z = 0.25
            publisher.publish(move)
        prev_orient_diff = orient_diff
    move.angular.z = 0
    publisher.publish(move)
    rospy.sleep(3)

def flood_fill(maze, goal_x, goal_y):
    flood_array = []
    for i in range(len(maze)):
        row = []
        for j in range(len(maze[i])):
            row.append(-1)
        flood_array.append(row)

    dist = 0
    flood_array[goal_y][goal_x] = dist
    while any(-1 in row for row in flood_array):
        for i in range(len(flood_array)):
            for j in range(len(flood_array[i])):
                if flood_array[i][j] == dist:
                    #up
                    if i != 0:
                        if flood_array[i-1][j] == -1 and not has_wall(maze[i][j], 0) and not has_wall(maze[i-1][j], 2):
                            flood_array[i-1][j] = dist + 1
                    #down
                    if i != len(flood_array) - 1:
                        if flood_array[i+1][j] == -1 and not has_wall(maze[i][j], 2) and not has_wall(maze[i+1][j], 0):
                            flood_array[i+1][j] = dist + 1
                    #left
                    if j != 0:
                        if flood_array[i][j-1] == -1 and not has_wall(maze[i][j], 3) and not has_wall(maze[i][j-1], 1):
                            flood_array[i][j-1] = dist + 1
                    #right
                    if j != len(flood_array[i]) - 1:
                        if flood_array[i][j+1] == -1 and not has_wall(maze[i][j], 1) and not has_wall(maze[i][j+1], 3):
                            flood_array[i][j+1] = dist + 1
        dist += 1
        if dist > 100:
            break

    print("flood:")
    print_array(flood_array)

    return flood_array

def print_array(array):
    for row in array:
        print(row)

def update_maze(maze, maze_x, maze_y):
    if maze[maze_y][maze_x] == 0:
        wall_type = which_wall()
        maze[maze_y][maze_x] = wall_type
    
    print("maze:")
    print_array(maze)
    
    return maze

def find_next_cell(flood_array, maze_array, maze_x, maze_y):
    orients = [0,1,2,3]
    new_orient = -1
    flood_value = 1000
    for orient in orients:
        #up
        if orient == 0 and maze_y != 0:
            if flood_array[maze_y-1][maze_x] < flood_value and not has_wall(maze[maze_y][maze_x], orient):
                new_orient = orient
                flood_value = flood_array[maze_y-1][maze_x]
        #down
        if orient == 2 and maze_y != len(flood_array) - 1:
            if flood_array[maze_y+1][maze_x] < flood_value and not has_wall(maze[maze_y][maze_x], orient):
                new_orient = orient
                flood_value = flood_array[maze_y+1][maze_x]
        #left
        if orient == 3 and maze_x != 0:
            if flood_array[maze_y][maze_x-1] < flood_value and not has_wall(maze[maze_y][maze_x], orient):
                new_orient = orient
                flood_value = flood_array[maze_y][maze_x-1]
        #right
        if orient == 1 and maze_x != len(flood_array[0]) - 1:
            if flood_array[maze_y][maze_x+1] < flood_value and not has_wall(maze[maze_y][maze_x], orient):
                new_orient = orient
                flood_value = flood_array[maze_y][maze_x+1]

    return new_orient

def move(publisher, new_orient, maze_x, maze_y):
    if new_orient == 0:
        maze_y -= 1
    if new_orient == 1:
        maze_x += 1
    if new_orient == 2:
        maze_y += 1
    if new_orient == 3:
        maze_x -= 1

    turn(publisher, new_orient)
    go_forward(publisher)

    return maze_x, maze_y


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
goal_x, goal_y = 6, 0
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

while maze_x != goal_x or maze_y != goal_y:
    maze = update_maze(maze, maze_x, maze_y)
    flood = flood_fill(maze, goal_x, goal_y)
    new_orient = find_next_cell(flood, maze, maze_x, maze_y)
    maze_x, maze_y = move(pub_vel, new_orient, maze_x, maze_y)
