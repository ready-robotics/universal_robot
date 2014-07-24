#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import rospy
# QT
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

class URStatus(QWidget):
    def __init__(self,app):
        super(URStatus,self).__init__()
        self.app_ = app
        self.servo_enable = False
        self.listener_ = TransformListener()
        self.broadcaster_ = TransformBroadcaster()

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('ur_driver') + '/marker_teleop.ui'
        uic.loadUi(ui_path, self)
        self.show()

        self.target_pub = rospy.Publisher('/ur5_command_pose',PoseStamped)

        # Load Settings
        self.settings = QSettings('marker_teleop_settings.ini', QSettings.IniFormat)
        self.settings.setFallbacksEnabled(False) 
        self.resize( self.settings.value('size', QSize(350, 350), type=QSize) )
        self.move(self.settings.value('pos', QPoint(50, 50), type=QPoint))

        # Set up ros_ok watchdog timer to handle termination and ctrl-c
        self.ok_timer_ = QTimer(self)
        self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
        self.ok_timer_.start(100)

        self.update_timer_ = QTimer(self)
        self.connect(self.update_timer_, QtCore.SIGNAL("timeout()"),self.update)
        self.update_timer_.start(100)

        self.enable_on_btn.clicked.connect(self.enable)
        self.enable_off_btn.clicked.connect(self.disable)
        self.servo_to_btn.clicked.connect(self.servo_to_pose)

        self.latest_pose = self.get_tcp_pose()
        self.initial_pose = self.get_tcp_pose()

    def enable(self):
        if self.servo_enable == True:
            rospy.logwarn("Already Servoing")
        else:
            self.servo_enable = True
            self.latest_pose = self.get_tcp_pose()
            rospy.logwarn("Continuous Servo Enabled")

    def disable(self):
        if self.servo_enable == False:
            rospy.logwarn("Not Servoing")
        else:
            self.servo_enable = False
            self.latest_pose = self.get_tcp_pose()
            rospy.logwarn("Continuous Servo Disabled")

    def servo_to_pose(self):
        self.latest_pose = self.get_tcp_pose()
        rospy.wait_for_service('/ur_driver/servo_to_pose')
        try:
            pose_servo_proxy = rospy.ServiceProxy('/ur_driver/servo_to_pose',servo_to_pose)
            F_target = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
            F_ur_base_to_base_link = tf_c.fromTf(self.listener_.lookupTransform('/ur_base','/base_link',rospy.Time(0)))
            F_target_trans = F_target * F_ur_base_to_base_link.Inverse()
                
            msg = ur_driver.srv.servo_to_poseRequest()
            msg.target = tf_c.toMsg(F_target_trans)
            msg.accel = .3
            msg.vel = .1
            # Send Servo Command
            result = pose_servo_proxy(msg)
            print str(result.ack)

        except rospy.ServiceException, e:
            print e

    def update(self):
        if self.servo_enable == True:
            try:
                F_target = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
                F_ee = tf_c.fromTf(self.listener_.lookupTransform('/world','/endpoint',rospy.Time(0)))
                F_reported_ee = tf_c.fromMsg(self.latest_pose)
                F_ur_base_to_base_link = tf_c.fromTf(self.listener_.lookupTransform('/ur_base','/base_link',rospy.Time(0)))
                self.broadcaster_.sendTransform(tuple(F_ur_base_to_base_link.p),tuple(F_ur_base_to_base_link.M.GetQuaternion()),rospy.Time.now(), '/other_base','/world')

                F_calc = F_ee*F_ur_base_to_base_link.Inverse()

                self.broadcaster_.sendTransform(tuple(F_reported_ee.p),tuple(F_reported_ee.M.GetQuaternion()),rospy.Time.now(), '/endpoint_reported','/world')
                self.broadcaster_.sendTransform(tuple(F_calc.p),tuple(F_calc.M.GetQuaternion()),rospy.Time.now(), '/endpoint_calc','/world')
                
                self.broadcaster_.sendTransform(tuple(F_target.p),tuple(F_target.M.GetQuaternion()),rospy.Time.now(), '/target','/world')

                F_target_in_base = F_target*F_ur_base_to_base_link
                self.broadcaster_.sendTransform(tuple(F_target_in_base.p),tuple(F_target_in_base.M.GetQuaternion()),rospy.Time.now(), '/target_in_base','/world')

                self.target_pose = tf_c.toMsg(F_target)
                cmd = PoseStamped()
                cmd.pose.position = self.target_pose.position
                cmd.pose.orientation = self.initial_pose.orientation

                self.target_pub.publish(cmd)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(str(e))
        else:
            pass
            # rospy.logwarn('Not in servo mode')

    def get_tcp_pose(self):
        rospy.wait_for_service('/ur_driver/get_tcp_pose')
        try:
            tcp_service = rospy.ServiceProxy('/ur_driver/get_tcp_pose',get_tcp_pose)
            result = tcp_service('')
            return result.current_pose
        except rospy.ServiceException, e:
            print e

# Widget Functions ------------------------------------------------------------#

    def closeEvent(self, event):
        print 'saving info'
        self.settings.setValue('size', self.size())
        self.settings.setValue('pos', self.pos())
        self.settings.sync()
        event.accept()

    def check_ok(self):
        if rospy.is_shutdown():
            self.close()
            self.app_.exit()

# MAIN #########################################################################
if __name__ == '__main__':
    rospy.init_node('ur_status_interface',anonymous=True)
    app = QApplication(sys.argv)
    status_app = URStatus(app)
    # Running
    app.exec_()
    # Done




















