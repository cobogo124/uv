#!/usr/bin/env python3

import rospy
import math
import angles
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point
from uvcontroller.msg import Sector
from std_msgs.msg import Bool


rospy.init_node("main_node")

goal = (1,1)
initialReached = False
rate = rospy.Rate(10)
toleratedDistance = 0.2
command = Twist()
humanSector = Sector()
sectorsectorPath = []
currentPosition = Pose()

sec_pub = rospy.Publisher("/human/sector", Sector, queue_size=10)
vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

def getHumanSector(current: Sector):
    global humanSector
    humanSector = current

def euclideanDistance() :
    return math.sqrt(pow((goal[0]-currentPosition.x), 2) + pow((goal[1]-currentPosition.y), 2))

def getLocation(current: Pose):
    global currentPosition
    currentPosition = current

def sectorPathFindRest(startingx, startingy, trueForRight):
    goRight = trueForRight
    currentx = startingx
    currenty = startingy
    while currentx != 4 and currenty != 4:
        if goRight:
            for i in range(currentx, 5):
                sector = Sector()
                sector.x = i
                sector.y = currenty
                sectorsectorPath.append(sector)
                currentx = i
            currenty += 1
            goRight = not goRight
        else:
            for i in reversed(range(currentx)):
                sector = Sector()
                sector.x = i
                sector.y = currenty
                sectorsectorPath.append(sector)
                currentx = i
            currenty += 1
            goRight = not goRight

def sectorPathFindUpTo(endx, endy):
    goRight = True
    currentx = 0
    currenty = 0
    while currentx != endx and currenty != endy:
        if goRight:
            for i in range(5):
                sector = Sector()
                sector.x = i
                sectory = currenty
                sectorPath.append(sector)
                currentx = i
            sectory += 1
            goRight = not goRight
        else:
            for i in reversed(range(currentx)):
                sector = Sector()
                sector.x = i
                sector.y = currenty
                sectorPath.append(sector)
                currentx = i
            currenty += 1
            goRight = not goRight

def sectorPathFindForHuman():
    minx, miny = 0
    maxx, maxy = 4
    currentx = 0
    currenty = 0
    if humanSector.x == maxx:
        if humanSector.y == miny:
            sectorPathFindUpTo(3, 0)
            for i in range(humanSector.x - 1):
                sector = Sector()
                sector.x = i
                sector.y = currenty
                sectorPath.append(sector)
            sector = Sector()
            sector.x = humanSector.x - 1
            sector.y = currrent.y + 1
            seectorPath.append(sector)
            sector.x = maxx
            currentx = maxx
            sector.y  = currrent.y + 1
            sectorPath.append(sector)
            currenty += 1
        sectorPathFindRest(currentx, currenty, False)
    elif humanSector.y == miny:
        if humanSector.x != maxx:
            sectorPathFindUpTo(humnanSector)

rospy.Subscriber("/turtle1/pose", Pose, getLocation)
rospy.Subscriber("/human/sector", Sector, getHumanSector)

while not initialReached:
    if euclideanDistance() <= 0.2:
        initialReached = True
        command.angular.z = 0
        command.linear.x = 0
    else:
        diff = (goal[0] - currentPosition.x, goal[1] - currentPosition.y)
        goal_orientation = math.atan2(diff[1], diff[0])
        angle_diff = angles.shortest_angular_distance(currentPosition.theta, goal_orientation)

        if angle_diff < -0.087 or angle_diff > 0.087:
            command.angular.z = -1
            command.linear.x = 0
        else:
            command.angular.z = 0
            command.linear.x = 3

    vel.publish(command)
    rate.sleep()

# while not rospy.is_shutdown():
#     sectorPathFind()
