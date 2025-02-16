#!/usr/bin/env python3

import rospy
import math
import angles
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point
from uvcontroller.msg import Sector
from std_msgs.msg import Bool

rospy.init_node("main_node")

initialx, initialy = 1, 1
initialReached = False
rate = rospy.Rate(10)
command = Twist()

currentPosition = Pose()

sec_pub = rospy.Publisher("/human/sector", Sector, queue_size=10)
vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

def get_square_coordinates(x, y):
    square_x = int(x // 2)
    square_y = int(y // 2)
    return (square_x, square_y)

def getLocation(current: Pose):
    global currentPosition
    currentPosition = (current.x, current.y, current.theta)

rospy.Subscriber("/turtle1/pose", Pose, getLocation)

while initialReached == False:
    diff = (1 - currentPosition[0], 1 - currentPosition[1])
    goal_orientation = math.atan2(diff[1], diff[0])

    angle_diff = angles.shortest_angular_distance(currentPosition[2], goal_orientation)

    if angle_diff < -0.087 or angle_diff > 0.087:
        command.angular.z = -1
        command.linear.x = 0
    else:
        command.angular.z = 0
        command.linear.x = 3

    vel.publish(command)
    rate.sleep()

while not rospy.is_shutdown():
    if not initialReached:
        goToInitial()
    else:
        print("not yet")
    rate.sleep()
