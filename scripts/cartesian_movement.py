#!/usr/bin/env python
import roslib; roslib.load_manifest('moveit_msgs')
import actionlib
import time
import sys
import rospy
# from GetCartesianPath.srv import *
from control_msgs.msg import *
from moveit_msgs.srv import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from cartesian_trajectory_msgs.msg import *

class CartPusher(object):
    def __init__(self):
        self.get_cartesian_path = rospy.ServiceProxy('get_cartesian_path', GetCartesianPath)
        rospy.wait_for_service('get_cartesian_path')
        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)
        rospy.Subscriber("dmp", cartesian_trajectory_msgs.msg.CartesianTrajectory, self.dmp_callback)

    def joint_state_callback(msg):
        # get a joint state message
        # store it
        newest_joint_state = msg.data
        rospy.loginfo(msg.data)

    def dmp_callback(msg):
        incoming_dmp_msg_pose = msg.data.points.poses
        incoming_dmp_time_from_start = msg.data.points.time_from_start

        cart_path_request = moveit_msgs.srv.GetCartesianPathRequest()
        cart_path_request.start_state = newest_joint_state # Copy the latest joint state into the get_cartesian_path message
        cart_path_request.group_name = manipulator # or arm if we are using the ur5_robotiq_2_fingered
        # cart_path_request.link_name = # Optional name of IK link for which waypoints are specified.  If not specified, the tip of the group (which is assumed to be a chain) is assumed to be the link  
        cart_path_request.waypoints = msg.data.dmp.points.poses
        cart_path_request.max_step = 2
        cart_path_request.jump_threshold = 5
        cart_path_request.avoid_collisions = False
        # cart_path_request.path_constraints = # Specify additional constraints to be met by the Cartesian path

        # Pass the dmp to get_cartesian_path
        try:
            resp = self.get_cartesian_path(cart_path_request) # I want to stuff get_cartesian_path into JointTrajectoryAction
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # output from get_cartesian_path:
        output_joint_state = resp.start_state.joint_state  # The state at which the computed path starts
        output_attached_collision_objects = resp.start_state.attached_collision_objects
        output_is_diff = resp.is_diff
        output_joint_trajectory_positions = resp.solution.joint_trajectory.points.positions # The computed solution trajectory, for the desired group, in configuration space
        output_joint_trajectory_velocities = resp.solution.joint_trajectory.points.velocities
        output_joint_trajectory_duration = resp.solution.joint_trajectory.points.time_from_start
        output_fraction = resp.fraction # If the computation was incomplete, this value indicates the fraction of the path that was in fact computed (nr of waypoints traveled through)
        output_error_code = resp.error_code # The error code of the computation

        # The following code
        # 1. Reads joint trajectory out of resp and puts it into a follow joint trajectory action
        # 2. Then it calls the follow joint trajectory action

        JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [ JointTrajectoryPoint(
            positions=output_joint_trajectory_positions, 
            velocities=output_joint_trajectory_velocities, 
            time_from_start=output_joint_trajectory_duration)]
        client.send_goal(g)
        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise

if __name__ == "__main__":
    global newest_joint_state
    global client
    try:
        rospy.init_node('cartesian_movement_listener', anonymous=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        cp = CartPusher()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

    rospy.spin()