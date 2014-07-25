#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
import PyKDL
import ur_driver; from ur_driver.srv import *
import tf_conversions as tf_c
from copy import deepcopy
if __name__ == '__main__':
    current_pose = None

    rospy.init_node("test_servo", anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/ur5_command_pose',PoseStamped)

    # print ''
    # rospy.wait_for_service('/ur_driver/get_tcp_pose')
    # try:
    #     tcp_service = rospy.ServiceProxy('/ur_driver/get_tcp_pose',get_tcp_pose)
    #     result = tcp_service('')
    #     print str(result.current_pose)
    #     print str(result.current_euler)
    #     current_pose = result.current_pose
    # except rospy.ServiceException, e:
    #     print e

    # rospy.sleep(3)
    # print ''

    # rospy.wait_for_service('/ur_driver/servoc')
    # try:
    #     servoc_prox = rospy.ServiceProxy('/ur_driver/servoc',servoc)

    #     pose = current_pose
    #     pose.position.z += .01

    #     msg = ur_driver.srv.servocRequest()
    #     msg.target = pose
    #     msg.accel = .3
    #     msg.vel = .1

    #     result = servoc_prox(msg)
    #     print str(result.ack)
    # except rospy.ServiceException, e:
    #     print e

    # Command Pose Sequence over publisher

    rospy.wait_for_service('/ur_driver/get_tcp_pose')
    try:
        tcp_service = rospy.ServiceProxy('/ur_driver/get_tcp_pose',get_tcp_pose)
        result = tcp_service('')
        print str(result.current_pose)
        print str(result.current_euler)
        current_pose = result.current_pose
    except rospy.ServiceException, e:
        print e

    rospy.sleep(1)

    pose_cur = current_pose
    val = 1
    poses = []
    for i in range(10):
        p = deepcopy(pose_cur)
        p.position.x+=pow(val,2)/300.0
        p.position.z += val/15.0
        poses.append(p)
        val += 1
        # print p.position.x
        print p.position.z

    for i in range(10):
        cmd = PoseStamped()
        cmd.pose=poses[i]
        pub.publish(cmd)
        rospy.sleep(1)