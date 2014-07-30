#!/usr/bin/env python
import roslib
roslib.load_manifest('ur_driver')
import rospy
# import os
# import sys
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtCore import Qt
# from xdot.xdot_qt import DotWidget
# import std_msgs.msg
import rospkg
import tf
import QtCore

from ur_status_interface import URStatus
from marker_teleop_interface import URMarkerTeleop


class URStatusPanel(Plugin):

    def __init__(self, context):
        super(URStatusPanel, self).__init__(context)

        self.setObjectName('UR5 Status Panel')
        # Create QWidget
        self._widget = URStatus()
        self._widget.setObjectName('UR5StatusPanel')
        self._widget.setWindowTitle('UR5 Status Panel')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        palette = QPalette ()
        palette.setColor(QPalette.Background, Qt.white)
        self._widget.setPalette(palette)

    def shutdown_plugin(self):
        # unregister all publishers here
        self.pub_list.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

class URMarkerTeleopPanel(Plugin):

    def __init__(self, context):
        super(URMarkerTeleopPanel, self).__init__(context)

        self.setObjectName('UR5 Marker Teleop Panel')
        # Create QWidget
        self._widget = URMarkerTeleop()
        self._widget.setObjectName('UR5MarkerTeleopPanel')
        self._widget.setWindowTitle('UR5 Marker Teleop Panel')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        palette = QPalette ()
        palette.setColor(QPalette.Background, Qt.white)
        self._widget.setPalette(palette)

    def shutdown_plugin(self):
        # unregister all publishers here
        self.pub_list.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


