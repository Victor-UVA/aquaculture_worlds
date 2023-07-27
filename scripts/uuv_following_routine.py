#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Point
from uuv_control_msgs.msg import Waypoint
from uuv_control_msgs.srv import GoTo, GoToRequest

def createNewGoToRequest(goalCoords):
    point = Point()
    point.x = goalCoords[0]
    point.y = goalCoords[1]
    point.z = goalCoords[2]

    waypoint = Waypoint()
    waypoint.point = point
    waypoint.max_forward_speed = 1.0
    waypoint.heading_offset = 0.0
    waypoint.use_fixed_heading = True
    waypoint.radius_of_acceptance = 1.0

    message = GoToRequest()
    message.waypoint = waypoint
    message.max_forward_speed = 1.0
    message.interpolator = 'CUBIC'

    return message

def withinDistanceLimits(goalCoords, currentCoords, limit):
    distance = sum([(goalCoords[i] - currentCoords[i])**2 for i in range(3)])**0.5
    return distance <= limit

if __name__ == '__main__':
    rospy.init_node("uuv_following_node")

    wamv_listener = tf.TransformListener()
    rexrov2_listener = tf.TransformListener()

    rospy.sleep(1)
    rospy.wait_for_service('/rexrov2/go_to')

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            wamv_trans, wamv_rot = wamv_listener.lookupTransform("world", "wamv/base_link", rospy.Time(0))
            rexrov2_trans, rexrov2_rot = rexrov2_listener.lookupTransform("world", "rexrov2/base_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        goalCoords = [wamv_trans[0], wamv_trans[1], -5.0]
        print("Goal:", goalCoords)
        if not withinDistanceLimits(goalCoords, rexrov2_trans, 1.0):
            message = createNewGoToRequest(goalCoords)

            try:
                go_to = rospy.ServiceProxy('/rexrov2/go_to', GoTo)
                response = go_to(message)
                print(response)
            except rospy.ServiceException as e:
                print('Service call failed: %s' % e)