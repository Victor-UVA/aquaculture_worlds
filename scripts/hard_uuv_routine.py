#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from uuv_control_msgs.srv import InitWaypointsFromFile, InitWaypointsFromFileRequest
import yaml

if __name__ == '__main__':
    filename = rospy.get_param('/aqua_uuv_routine/uuv_waypoint_filename', '')
    if len(filename) == 0:
        print('Must set filename param')
        quit()
    else:
        print(filename)
    with open(filename, 'r') as wp_file:
        wps = yaml.load(wp_file, Loader=yaml.FullLoader)
    
    message = InitWaypointsFromFileRequest()
    message.start_now = True
    message.filename = String(filename)
    message.interpolator = String('CUBIC')
    
    rospy.sleep(1)
    rospy.wait_for_service('/rexrov2/init_waypoints_from_file')
    
    try:
        init_waypoints = rospy.ServiceProxy('/rexrov2/init_waypoints_from_file', InitWaypointsFromFile)
        response = init_waypoints(message)
        print(response)
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)
