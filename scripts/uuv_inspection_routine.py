#!/usr/bin/env python3

import numpy as np
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Point
from uuv_control_msgs.msg import Waypoint
from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest

#circle parameters
center_axis = [0, 0]
radius = 30
max_depth = 25
min_depth = 3

#path parameters
n = 12
depth_n = 10
approach_speed = 1.0
traversal_speed = 0.9

def createNewWaypointSetRequest(coordList):
    message = InitWaypointSetRequest()
    message.start_now = True
    message.max_forward_speed = 1.0
    message.heading_offset = 0.0
    message.interpolator = String('CUBIC')

    waypointList = []
    for coord in coordList:
        waypointList.append(createNewWaypoint(coord))

    message.waypoints = waypointList

    return message

def createNewWaypoint(goalCoords):
    point = Point()
    point.x = goalCoords[0]
    point.y = goalCoords[1]
    point.z = goalCoords[2]

    waypoint = Waypoint()
    waypoint.point = point
    waypoint.max_forward_speed = 1.0
    waypoint.heading_offset = 0.0
    waypoint.use_fixed_heading = True
    waypoint.radius_of_acceptance = 0.0

    return waypoint

def withinDistanceLimits(goalCoords, currentCoords, limit):
    distance = sum([(goalCoords[i] - currentCoords[i])**2 for i in range(3)])**0.5
    return distance <= limit

def constructGoalCoords():
    goalCoords = []
    for i in range(n): #construct main path
        x, y = pathCoords[i * 2]
        for v in vertical:
            goalCoords.append([x, y, -v, 0])

        x, y = pathCoords[(i * 2) + 1]
        for v in vertical[::-1]:
            goalCoords.append([x, y, -v, 0])
    
    #close path and return to the start
    goalCoords.append([pathCoords[0][0], pathCoords[0][1], -vertical[0], 0])
    goalCoords.append([robotCoord[0], robotCoord[1], robotCoord[2], 0])

    return goalCoords

def getPathCoord(angle):
    x = center_axis[0] + (radius * np.cos(angle))
    y = center_axis[1] + (radius * np.sin(angle))
    return [x, y]

if __name__ == '__main__':
    rospy.init_node("uuv_pathing_test_node")

    wamv_listener = tf.TransformListener()
    rexrov2_listener = tf.TransformListener()

    rospy.sleep(1)
    rospy.wait_for_service('/rexrov2/start_waypoint_list')

    """goalCoords = [
        [32, 32, -10],
        [40, 50, -30],
        [36, 36,  -5],
        [32, 32, -20]
    ]"""

    try:
        wamv_trans, wamv_rot = wamv_listener.lookupTransform("world", "wamv/base_link", rospy.Time(0))
        rexrov2_trans, rexrov2_rot = rexrov2_listener.lookupTransform("world", "rexrov2/base_link", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Could not find transforms")

    #robot parameters
    robotCoord = rexrov2_trans
    robotCoord2d = robotCoord[:2]

    start_angle = np.arctan(robotCoord2d[1]/robotCoord2d[0])
    if robotCoord2d[0] < 0:
        start_angle += np.pi

    slices = np.linspace(start_angle, start_angle + 2 * np.pi, n * 2, endpoint=False)
    vertical = np.linspace(min_depth, max_depth, depth_n)

    pathCoords = np.array([getPathCoord(angle) for angle in slices])

    goalCoords = constructGoalCoords()
    message = createNewWaypointSetRequest(goalCoords)

    try:
        start_waypoint_list = rospy.ServiceProxy('/rexrov2/start_waypoint_list', InitWaypointSet)
        response = start_waypoint_list(message)
        print(response)
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)