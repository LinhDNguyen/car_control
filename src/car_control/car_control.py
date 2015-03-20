# Copyright (c) 2014, Linh Nguyen <nvl1109@gmail.com>
# All rights reserved.

from __future__ import division
import os
import rospkg

import rospy
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget
from rqt_gui_py.plugin import Plugin

from pi_dc_motor.msg import Motor


class CarControlWidget(QWidget):
    def __init__(self, parent):
        super(CarControlWidget, self).__init__(parent=parent)

        self.initUi()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Left:
            rospy.logdebug('Left Arrow Pressed')
        elif key == Qt.Key_Right:
            rospy.logdebug('Right Arrow Pressed')
        elif key == Qt.Key_Up:
            rospy.logdebug('Up Arrow Pressed')
        elif key == Qt.Key_Down:
            rospy.logdebug('Down Arrow Pressed')

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_Left:
            rospy.logdebug('Left Arrow Released')
        elif key == Qt.Key_Right:
            rospy.logdebug('Right Arrow Released')
        elif key == Qt.Key_Up:
            rospy.logdebug('Up Arrow Released')
        elif key == Qt.Key_Down:
            rospy.logdebug('Down Arrow Released')

    def mouseMoveEvent(self, event):
        pos = event.pos()

        rospy.logdebug("x: %d - y: %d" % (event.x(), event.y()))

    def initUi(self):
        self.setFixedHeight(600)
        self.setFixedWidth(600)
        # self.setMouseTracking(True)
        self.grabKeyboard()
        rospy.loginfo("CarControl: keyboard grabed. Start control your car.")

    def shutdown(self):
        self.releaseKeyboard()
        rospy.loginfo("CarControl: shutdown")


class CarControl(Plugin):
    def __init__(self, context):
        super(CarControl, self).__init__(context)
        self.setObjectName('CarControl')

        self._publisher = None

        self._widget = CarControlWidget(None)
        if context.serial_number() > 1:
            self._widget.setWindowTitle("Car Control" + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass








