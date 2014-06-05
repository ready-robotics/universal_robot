#!/usr/bin/env python
import roslib; roslib.load_manifest('moveit_msgs')
import actionlib
import time
import sys
import rospy
# from GetCartesianPath.srv import *
from control_msgs.msg import *
from moveit_msgs.srv import *
from moveit_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from moveit_commander import MoveGroupCommander

class CartPusher(object):
    def __init__(self):

        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)
        rospy.Subscriber("dmp", cartesian_trajectory_msgs.msg.CartesianTrajectory, self.dmp_callback)
        rospy.Subscriber("gripper",control_msgs.msg.GripperCommand,self.gripper_callback)
        print "waiting for service compute_cartesian_path"
        rospy.wait_for_service('compute_cartesian_path')
        self.get_cartesian_path = rospy.ServiceProxy('compute_cartesian_path', GetCartesianPath)
        print "found service compute_cartesian_path"

    def joint_state_callback(self,msg):
        # get a joint state message
        # store it
        my_object = RobotState()
        my_object.joint_state = JointState()
        my_object.joint_state.name = msg.name
        my_object.joint_state.position = msg.position
        my_object.joint_state.velocity = msg.velocity
        my_object.joint_state.effort = msg.effort
        self.newest_joint_state = my_object
        # rospy.loginfo(msg)

    def gripper_callback(self,msg):
        global client2
        gripper_position = msg.position
        gripper_max_effort = msg.max_effort

        if gripper_position == 0:
            joint_trajectory_positions = [0, 0, 0, 0] # The computed solution trajectory, for the desired group, in configuration space
        elif gripper_position == 1:
            joint_trajectory_positions= [2.4, 2.4, 0, 0]
        else:
            joint_trajectory_positions = [0, 0, 0, 0]
        joint_trajectory_velocities = [2, 2, 2, 2]
        joint_trajectory_duration = rospy.Duration.from_sec(3)

        # The following code
        # 1. Reads joint trajectory out of resp and puts it into a follow joint trajectory action
        # 2. Then it calls the follow joint trajectory action

        JOINT_NAMES = ['robotiq_85_left_inner_knuckle_joint', 
                        'robotiq_85_right_inner_knuckle_joint', 
                        'robotiq_85_left_finger_tip_joint', 
                        'robotiq_85_right_finger_tip_joint']

        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [ JointTrajectoryPoint(
            positions=joint_trajectory_positions, 
            velocities=joint_trajectory_velocities, 
            time_from_start=joint_trajectory_duration)]

        client2.send_goal(g)
        try:
            client2.wait_for_result()
        except KeyboardInterrupt:
            client2.cancel_goal()
            raise

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def dmp_callback(self,msg):
        global client
        # [point.poses[0] for point in msg.points]
        # incoming_dmp_msg_pose = msg.points[].poses[0]
        # incoming_dmp_time_from_start = msg.points[0].time_from_start

        cart_path_request = moveit_msgs.srv.GetCartesianPathRequest()
        cart_path_request.start_state = self.newest_joint_state # Copy the latest joint state into the get_cartesian_path message
        cart_path_request.group_name = "arm" # or arm if we are using the ur5_robotiq_2_fingered
        cart_path_request.link_name = "wrist_3_link"# Optional name of IK link for which waypoints are specified.  If not specified, the tip of the group (which is assumed to be a chain) is assumed to be the link  
        cart_path_request.waypoints = [point.poses[0] for point in msg.points]
        cart_path_request.max_step = 0.1
        cart_path_request.jump_threshold = 0
        cart_path_request.avoid_collisions = False
        # cart_path_request.path_constraints = # Specify additional constraints to be met by the Cartesian path
        print cart_path_request.waypoints
        # Pass the dmp to get_cartesian_path
        try:
            resp = self.get_cartesian_path(cart_path_request) # I want to stuff get_cartesian_path into JointTrajectoryAction
            print resp
            # output from get_cartesian_path:
            output_joint_state = resp.start_state.joint_state  # The state at which the computed path starts
            output_attached_collision_objects = resp.start_state.attached_collision_objects
            # output_is_diff = resp.is_diff
            #print resp.solution.joint_trajectory.points
            output_joint_trajectory_points = resp.solution.joint_trajectory.points
            # output_joint_trajectory_positions = resp.solution.joint_trajectory.points.positions # The computed solution trajectory, for the desired group, in configuration space
            # output_joint_trajectory_velocities = resp.solution.joint_trajectory.points.velocities
            # output_joint_trajectory_duration = resp.solution.joint_trajectory.points.time_from_start
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
            # g.trajectory.points = [ JointTrajectoryPoint(
            #     positions=output_joint_trajectory_positions, 
            #     velocities=output_joint_trajectory_velocities, 
            #     time_from_start=output_joint_trajectory_duration)]
            g.trajectory.points = output_joint_trajectory_points

            client.send_goal(g)
            try:
                client.wait_for_result()
            except KeyboardInterrupt:
                client.cancel_goal()
                raise

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == "__main__":
    global client
    try:
        rospy.init_node('cartesian_movement_listener', anonymous=True)
        client = actionlib.SimpleActionClient('ur5_robotiq_joint_limited/arm_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client2 = actionlib.SimpleActionClient('ur5_robotiq_joint_limited/hand_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        #client2.wait_for_server()
        print "Connected to server"
        cp = CartPusher()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

    rospy.spin()
