#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
import PyKDL
from ur_driver.srv import *
import tf_conversions as tf_c

if __name__ == '__main__':
    current_pose = None

    rospy.init_node("test_servo", anonymous=True, disable_signals=True)

    rospy.wait_for_service('/ur_driver/get_tcp_pose')
    try:
        tcp_service = rospy.ServiceProxy('/ur_driver/get_tcp_pose',get_tcp_pose)
        result = tcp_service('')
        print str(result.current_pose)
        print str(result.current_euler)
        current_pose = result.current_pose
        print tf_c.fromMsg(current_pose).M.GetRPY()

    except rospy.ServiceException, e:
        print e

    rospy.sleep(1)
    print ''

    rospy.wait_for_service('/ur_driver/movel')
    try:
        movel_service = rospy.ServiceProxy('/ur_driver/movel',movel)
        result = movel_service(current_pose)
        print str(result.ack)
    except rospy.ServiceException, e:
        print e

    rospy.sleep(1)


    rospy.wait_for_service('/ur_driver/free_drive')
    try:
        free_drive_service = rospy.ServiceProxy('/ur_driver/free_drive',free_drive)
        result = free_drive_service(True)
        print str(result.ack)
    except rospy.ServiceException, e:
        print e

    rospy.sleep(5)

    rospy.wait_for_service('/ur_driver/free_drive')
    try:
        free_drive_service = rospy.ServiceProxy('/ur_driver/free_drive',free_drive)
        result = free_drive_service(False)
        print str(result.ack)
    except rospy.ServiceException, e:
        print e


