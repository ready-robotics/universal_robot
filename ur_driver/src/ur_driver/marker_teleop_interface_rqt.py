#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import rospy
# QT
from qt_gui.plugin import Plugin
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Other
import actionlib
from std_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
import PyKDL
import ur_driver
from ur_driver.srv import *
import tf; from tf import *
import tf_conversions as tf_c
import rospkg

class URMarkerTeleopPanel(Plugin):
    def __init__(self,context):
        super(URMarkerTeleopPanel,self).__init__(context)

        self.setObjectName('UR5 Marker Teleop Panel')
        self._widget = QWidget()

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('ur_driver') + '/marker_teleop.ui'
        uic.loadUi(ui_path, self._widget)
        self._widget.setObjectName('UR5MarkerTeleopPanel')
        self._widget.setWindowTitle('UR5 Marker Teleop Panel')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Parameters
        self.target_pub = rospy.Publisher('/ur5_command_pose',PoseStamped)
        self.servo_enable = False
        self.listener_ = TransformListener()
        self.broadcaster_ = TransformBroadcaster()

        self.update_timer_ = QTimer(self)
        self.connect(self.update_timer_, QtCore.SIGNAL("timeout()"),self.update)
        self.update_timer_.start(100)

        self._widget.enable_on_btn.clicked.connect(self.enable)
        self._widget.enable_off_btn.clicked.connect(self.disable)
        self._widget.servo_to_btn.clicked.connect(self.servo_to_pose)

        self.initial_pose = self.get_tcp_pose()
        if self.initial_pose == None:
            rospy.logwarn('WARNING <<< DID NOT HEAR FROM ROBOT')
        else:
            rospy.logwarn('MARKER TELEOP INTERFACE READY')

    def enable(self):
        self.initial_pose = self.get_tcp_pose()
        if self.initial_pose == None:
            rospy.logwarn('WARNING <<< DID NOT HEAR FROM ROBOT')
            return

        if self.servo_enable == True:
            rospy.logwarn("Already Servoing")
        else:
            self.servo_enable = True
            rospy.logwarn("Continuous Servo Enabled")
            self._widget.enabled_label.setText('ENABLED')
            self._widget.enabled_label.setStyleSheet('color:#ffffff;background-color:#ADE817')

    def disable(self):
        self.initial_pose = self.get_tcp_pose()
        if self.initial_pose == None:
            rospy.logwarn('WARNING <<< DID NOT HEAR FROM ROBOT')
            return
        if self.servo_enable == False:
            rospy.logwarn("Not Servoing")
        else:
            self.servo_enable = False
            rospy.logwarn("Continuous Servo Disabled")
            self._widget.enabled_label.setText('DISABLED')
            self._widget.enabled_label.setStyleSheet('color:#ffffff;background-color:#FF9100')

    def servo_to_pose(self):
        self.initial_pose = self.get_tcp_pose()
        if self.initial_pose == None:
            rospy.logwarn('WARNING <<< DID NOT HEAR FROM ROBOT')
            return
        rospy.wait_for_service('/ur_driver/ServoToPose')
        try:
            pose_servo_proxy = rospy.ServiceProxy('/ur_driver/ServoToPose',ServoToPose)
            
            F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
            F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/target_frame',rospy.Time(0)))
            F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
            F_command = F_base_world.Inverse()*F_target_world
                
            msg = ur_driver.srv.ServoToPoseRequest()
            msg.target = tf_c.toMsg(F_command)
            msg.accel = .7
            msg.vel = .3
            # Send Servo Command
            rospy.logwarn('Single Servo Move Started')
            result = pose_servo_proxy(msg)
            rospy.logwarn('Single Servo Move Finished')
            rospy.logwarn(str(result.ack))

        except rospy.ServiceException, e:
            print e

    def update(self):
        if self.servo_enable == True:
            try:
                F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
                F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/target_frame',rospy.Time(0)))
                F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
                F_command = F_base_world.Inverse()*F_target_world

                cmd = PoseStamped()
                cmd.pose = tf_c.toMsg(F_command)
                self.target_pub.publish(cmd)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(str(e))
        else:
            pass
            # rospy.logwarn('Not in servo mode')

    def get_tcp_pose(self):
        try:
            rospy.wait_for_service('/ur_driver/GetTcpPose',1)
        except rospy.ROSException as e:
            rospy.logwarn(e)
            return None
        try:
            tcp_service = rospy.ServiceProxy('/ur_driver/GetTcpPose',GetTcpPose)
            result = tcp_service('')
            return result.current_pose
        except rospy.ServiceException, e:
            rospy.logwarn(e)
            return None

    def shutdown_plugin(self):
        # unregister all publishers here
        self.update_timer_.stop()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass