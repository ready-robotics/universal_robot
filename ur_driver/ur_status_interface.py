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
import tf_conversions as tf_c
import rospkg


class URStatus(QWidget):
    def __init__(self,app):
        super(URStatus,self).__init__()
        self.app_ = app
        self.setMinimumWidth(600)
        self.setMinimumHeight(600)
        self.freedrive = False


        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('ur_driver') + '/ur_status.ui'
        uic.loadUi(ui_path, self)
        self.show()
        # self.showMaximized()

        self.status_sub = rospy.Subscriber('/ur_driver/status',String,self.status_cb)

        # Set up ros_ok watchdog timer to handle termination and ctrl-c
        self.ok_timer_ = QTimer(self)
        self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
        self.ok_timer_.start(1000)

        self.enable_btn.clicked.connect(self.freedrive_enable)
        self.disable_btn.clicked.connect(self.freedrive_disable)

    def status_cb(self,msg):
        self.status = msg.data

    def freedrive_enable(self):
        print 'Enabling Freedrive...'
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
        elif self.status == 'FREEDRIVE':
            self.status_label.setText(str(self.status))
            self.status_label.setStyleSheet('color:#ffffff; background-color:#1AA5EB')
        elif self.status == 'DISCONNECTED':
            self.status_label.setText(str(self.status))
            self.status_label.setStyleSheet('color:#ffffff; background-color:#EB1A1D')

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




















