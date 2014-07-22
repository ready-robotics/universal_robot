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
from ur_driver.srv import *
import tf; from tf import *
import tf_conversions as tf_c
import rospkg


class URStatus(QWidget):
    def __init__(self,app):
        super(URStatus,self).__init__()
        self.app_ = app
        self.setMinimumWidth(600)
        self.setMinimumHeight(500)
        self.freedrive = False
        self.status = 'DISCONNECTED'
        self.servo_to_target = False
        self.listener_ = TransformListener()
        self.single_move = False

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('ur_driver') + '/ur_status.ui'
        uic.loadUi(ui_path, self)
        self.show()

        self.target_pub = rospy.Publisher('/ur5_command_pose',PoseStamped)

        self.status_sub = rospy.Subscriber('/ur_driver/status',String,self.status_cb)

        # Set up ros_ok watchdog timer to handle termination and ctrl-c
        self.ok_timer_ = QTimer(self)
        self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
        self.ok_timer_.start(100)

        self.update_timer_ = QTimer(self)
        self.connect(self.update_timer_, QtCore.SIGNAL("timeout()"),self.update)
        self.update_timer_.start(100)

        self.enable_btn.clicked.connect(self.freedrive_enable)
        self.disable_btn.clicked.connect(self.freedrive_disable)
        self.servo_enable_btn.clicked.connect(self.servo_enable)
        self.servo_disable_btn.clicked.connect(self.servo_disable)
        self.single_servo_check.stateChanged.connect(self.servo_mode_changed)

    def update(self):
        if self.servo_to_target == True:
            if self.status == 'SERVO':
                try:
                    T_target = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
                    self.target_pose = tf_c.toMsg(T_target)

                    # rospy.logwarn(self.target_pose)
                    # rospy.logwarn(self.initial_pose)

                    cmd = PoseStamped()
                    cmd.pose.position = self.target_pose.position
                    cmd.pose.orientation = self.initial_pose.orientation

                    self.target_pub.publish(cmd)
                    # rospy.logwarn('Sent command to robot')

                    if self.single_move == True:
                        self.servo_to_target = False

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn(str(e))
            else:
                rospy.logwarn('Not in servo mode')

    def status_cb(self,msg):
        self.status = msg.data

    def servo_enable(self):
        rospy.logwarn('enabling servo')
        if self.freedrive == False:
            if not self.servo_to_target:
                self.initial_pose = self.get_tcp_pose()
                # rospy.logwarn(self.initial_pose)
                if not self.initial_pose:
                    rospy.logerr('There was a problem getting the initial pose')
                    return
                else:
                    self.servo_to_target = True
                    return

    def servo_disable(self):
        rospy.logwarn('disabling servo')
        self.servo_to_target = False
        self.initial_pose = None
        r = self.stop_robot()
        rospy.logwarn("Stop Robot Result: " + str(r))

    def servo_mode_changed(self,val):
        if val == 2:
            rospy.logwarn('Single Servo Enabled')
            self.single_move = True
        elif val == 0:
            rospy.logwarn('Single Servo Disabled')
            self.single_move = False

    def freedrive_enable(self):
        print 'Enabling Freedrive...'
        # Always disable servoing
        self.servo_to_target = False
        if self.freedrive == False:
            try:
                rospy.wait_for_service('/ur_driver/free_drive',2)
            except rospy.ROSException as e:
                print 'Could not find freedrive service'
                return
            try:
                free_drive_service = rospy.ServiceProxy('/ur_driver/free_drive',free_drive)
                result = free_drive_service(True)
                print 'Service returned: ' + str(result.ack)
                self.freedrive = True
                self.enable_label.setText('ENABLED')
                self.enable_label.setStyleSheet('color:#ffffff;background-color:#ADE817')
            except rospy.ServiceException, e:
                print e
        else:
            print 'Freedrive is already enabled'

    def freedrive_disable(self):
        print 'Disabling Freedrive...'
        if self.freedrive == True:
            try:
                rospy.wait_for_service('/ur_driver/free_drive',2)
            except rospy.ROSException as e:
                print 'Could not find freedrive service'
                return
            try:
                free_drive_service = rospy.ServiceProxy('/ur_driver/free_drive',free_drive)
                result = free_drive_service(False)
                print 'Service returned: ' + str(result.ack)
                self.freedrive = False
                self.enable_label.setText('DISABLED')
                self.enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
            except rospy.ServiceException, e:
                print e
        else:
            print 'Freedrive is not enabled'

    def check_status(self):
        if self.status == 'IDLE':
            self.status_label.setText(str(self.status))
            self.status_label.setStyleSheet('color:#ffffff; background-color:#EBCF1A')
        elif self.status == 'SERVO':
            self.status_label.setText(str(self.status))
            self.status_label.setStyleSheet('color:#ffffff; background-color:#AFEB1A')
        elif self.status == 'TEACH':
            self.status_label.setText(str(self.status))
            self.status_label.setStyleSheet('color:#ffffff; background-color:#1AA5EB')
        elif self.status == 'DISCONNECTED':
            self.status_label.setText(str(self.status))
            self.status_label.setStyleSheet('color:#ffffff; background-color:#EB1A1D')

    def get_tcp_pose(self):
        rospy.wait_for_service('/ur_driver/get_tcp_pose')
        try:
            tcp_service = rospy.ServiceProxy('/ur_driver/get_tcp_pose',get_tcp_pose)
            result = tcp_service('')
            # rospy.logwarn(str(result.current_pose))
            return result.current_pose
        except rospy.ServiceException, e:
            print e

    def stop_robot(self):
        rospy.wait_for_service('/ur_driver/stop')
        try:
            stop_service = rospy.ServiceProxy('/ur_driver/stop',stop)
            result = stop_service('')
            return result
        except rospy.ServiceException, e:
            print e

# Widget Functions ------------------------------------------------------------#

    def closeEvent(self, event):
        # Finish up if needed
        event.accept()

    def clean_up(self):
        # Do some clean up here
        pass

    def check_ok(self):
        self.check_status()
        if rospy.is_shutdown():
            self.clean_up()
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




















