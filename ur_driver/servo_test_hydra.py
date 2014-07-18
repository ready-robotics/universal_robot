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
from razer_hydra.msg import *


def frame_from_transform(t):
    F = PyKDL.Frame()
    F.p = PyKDL.Vector(t.translation.x,t.translation.y,t.translation.z)
    F.M = PyKDL.Rotation.Quaternion(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    return F

class TestUR5Teleop(object):
    def __init__(self):
        current_pose = None

        rospy.init_node("test_servo", anonymous=True, disable_signals=True)
        pub = rospy.Publisher('/ur5_command_pose',PoseStamped)
        self.hydra_joy_sub = rospy.Subscriber("/hydra_joy",Joy,self.hydra_joy_cb)
        self.hydra_pos_sub = rospy.Subscriber("/hydra_calib",Hydra,self.hydra_pos_cb)
        self.F_hydra_current = Hydra()
        self.mode = 'IDLE'

        # get initial pose of robot
        self.F_robot_init = tf_c.fromMsg(get_pose())

        while not rospy.is_shutdown():
            self.update()

    def get_pose(self):
        rospy.wait_for_service('/ur_driver/get_tcp_pose')
        try:
            tcp_service = rospy.ServiceProxy('/ur_driver/get_tcp_pose',get_tcp_pose)
            result = tcp_service('')
            print str(result.current_pose)
            print str(result.current_euler)
            return result.current_pose
        except rospy.ServiceException, e:
            print e

    def hydra_pose_cb(self,msg):
        self.F_hydra_current = [frame_from_transform(msg.paddles[0].transform), frame_from_transform(msg.paddles[1].transform)]

    def hydra_joy_cb(msg):
        if self.mode == 'IDLE':
            if msg.buttons[9] == True:
                self.mode = 'SENDING'
                self.F_hydra_init = deepcopy(self.F_hydra_current) # TODO deep copy?
        elif self.mode == 'SENDING':
            if msg.buttons[9] == False:
                self.mode = 'IDLE'

    def update(self):
        


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


# MAIN
if __name__ == '__main__':
  tel = TestUR5Teleop()