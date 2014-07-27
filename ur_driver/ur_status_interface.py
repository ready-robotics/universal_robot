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
        self.freedrive = False
        self.servo = False
        self.status = 'DISCONNECTED'
        self.listener_ = TransformListener()

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('ur_driver') + '/ur_status.ui'
        uic.loadUi(ui_path, self)
        self.show()

        self.target_pub = rospy.Publisher('/ur5_command_pose',PoseStamped)
        self.status_sub = rospy.Subscriber('/ur_driver/status',String,self.status_cb)

        # Load Settings
        self.settings = QSettings('settings.ini', QSettings.IniFormat)
        self.settings.setFallbacksEnabled(False) 
        self.resize( self.settings.value('size', QSize(600, 500), type=QSize) )
        self.move(self.settings.value('pos', QPoint(50, 50), type=QPoint))

        # Set up ros_ok watchdog timer to handle termination and ctrl-c
        self.ok_timer_ = QTimer(self)
        self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
        self.ok_timer_.start(100)

        self.update_timer_ = QTimer(self)
        self.connect(self.update_timer_, QtCore.SIGNAL("timeout()"),self.update)
        self.update_timer_.start(100)

        self.freedrive_enable_btn.clicked.connect(self.freedrive_enable)
        self.freedrive_disable_btn.clicked.connect(self.freedrive_disable)
        self.servo_enable_btn.clicked.connect(self.servo_enable)
        self.servo_disable_btn.clicked.connect(self.servo_disable)

        self.gripper_open_btn.clicked.connect(self.gripper_open)
        self.gripper_close_btn.clicked.connect(self.gripper_close)

    def gripper_open(self):
        pass

    def gripper_close(self):
        pass

    def status_cb(self,msg):
        self.status = msg.data

    def servo_enable(self):
        rospy.logwarn('enabling servo')
        if self.freedrive == False:
            if self.servo == False:
                try:
                    rospy.wait_for_service('/ur_driver/ServoEnable',2)
                except rospy.ROSException as e:
                    print 'Could not find ServoEnable service'
                    self.msg_label.setText("NO SERVO_ENABLE SERVICE")
                    return
                try:
                    servo_enable_service = rospy.ServiceProxy('/ur_driver/ServoEnable',ServoEnable)
                    result = servo_enable_service(True)
                    self.servo = True
                    self.servo_enable_label.setText('ENABLED')
                    self.servo_enable_label.setStyleSheet('color:#ffffff;background-color:#ADE817')
                    self.msg_label.setText("SERVO ENABLED")
                except rospy.ServiceException, e:
                    print e
            else:
                self.msg_label.setText('SERVO ALREAD ENABLED')
        else:
            self.msg_label.setText("CANT SERVO, FREEDRIVE ENABLED")

    def servo_disable(self):
        rospy.logwarn('disabling servo')
        if self.freedrive == False:
            if self.servo == True:
                    try:
                        rospy.wait_for_service('/ur_driver/ServoEnable',2)
                    except rospy.ROSException as e:
                        print 'Could not find ServoEnable service'
                        self.msg_label.setText("NO SERVO_ENABLE SERVICE")
                        return
                    try:
                        servo_enable_service = rospy.ServiceProxy('/ur_driver/ServoEnable',ServoEnable)
                        result = servo_enable_service(False)
                        self.servo = False
                        self.servo_enable_label.setText('DISABLED')
                        self.servo_enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
                        self.msg_label.setText("SERVO DISABLED")
                    except rospy.ServiceException, e:
                        print e
            else:
                 self.msg_label.setText("SERVO IS NOT ENABLED")  
            # r = self.stop_robot()
            # rospy.logwarn("Stop Robot Result: " + str(r))
        else:
            self.msg_label.setText("CANT DISABLE SERVO, FREEDRIVE ENABLED")

    def freedrive_enable(self):
        rospy.logwarn('enabling freedrive')
        if self.servo == False:
            if self.freedrive == False:
                try:
                    rospy.wait_for_service('/ur_driver/FreeDrive',2)
                except rospy.ROSException as e:
                    print 'Could not find freedrive service'
                    return
                try:
                    free_drive_service = rospy.ServiceProxy('/ur_driver/FreeDrive',FreeDrive)
                    result = free_drive_service(True)
                    # print 'Service returned: ' + str(result.ack)
                    self.freedrive = True
                    self.freedrive_enable_label.setText('ENABLED')
                    self.freedrive_enable_label.setStyleSheet('color:#ffffff;background-color:#ADE817')
                    self.msg_label.setText("FREEDRIVE ENABLED")
                except rospy.ServiceException, e:
                    print e
            else:
                self.msg_label.setText("FREEDRIVE IS ALREADY ENABLED")
        else:
            self.msg_label.setText("CANT FREEDRIVE, SERVO ENABLED")

    def freedrive_disable(self):
        if self.freedrive == True:
            try:
                rospy.wait_for_service('/ur_driver/FreeDrive',2)
            except rospy.ROSException as e:
                print 'Could not find freedrive service'
                return
            try:
                free_drive_service = rospy.ServiceProxy('/ur_driver/FreeDrive',FreeDrive)
                result = free_drive_service(False)
                # print 'Service returned: ' + str(result.ack)
                self.freedrive = False
                self.freedrive_enable_label.setText('DISABLED')
                self.freedrive_enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
                self.msg_label.setText("FREEDRIVE DISABLED")
            except rospy.ServiceException, e:
                print e
        else:
            print 'Freedrive is not enabled'

    def check_status(self):
        if self.status == 'IDLE':
            self.mode_label.setText(str(self.status))
            self.mode_label.setStyleSheet('color:#ffffff; background-color:#EBCF1A')
        elif self.status == 'SERVO':
            self.mode_label.setText(str(self.status))
            self.mode_label.setStyleSheet('color:#ffffff; background-color:#AFEB1A')
        elif self.status == 'SERVO IDLE':
            self.mode_label.setText(str(self.status))
            self.mode_label.setStyleSheet('color:#ffffff; background-color:#B9C49B')
        elif self.status == 'TEACH':
            self.mode_label.setText(str(self.status))
            self.mode_label.setStyleSheet('color:#ffffff; background-color:#1AA5EB')
        elif self.status == 'DISCONNECTED':
            self.mode_label.setText(str(self.status))
            self.mode_label.setStyleSheet('color:#ffffff; background-color:#EB1A1D')

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
        print 'saving info'
        self.settings.setValue('size', self.size())
        self.settings.setValue('pos', self.pos())
        self.settings.sync()
        event.accept()

    def check_ok(self):
        self.check_status()
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




















