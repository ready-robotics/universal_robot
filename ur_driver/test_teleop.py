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