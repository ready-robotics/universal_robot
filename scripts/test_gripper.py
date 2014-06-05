#!/usr/bin/env python
# license removed for brevity
import rospy
from control_msgs.msg import *

def talker():
    pub = rospy.Publisher('gripper', GripperCommand, queue_size=10)
    rospy.init_node('gripper_talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # user_input = int(raw_input("Tell me gripper position number: "))
        msg = GripperCommand()
        # msg.position = user_input
        msg.position = 1
        msg.max_effort = 1
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass