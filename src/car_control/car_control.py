# Copyright (c) 2014, Linh Nguyen <nvl1109@gmail.com>
# All rights reserved.

from __future__ import division
import os
import rospkg

import rospy
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget
from rqt_gui_py.plugin import Plugin


class CarControlWidget(QWidget):
    def __init__(self, parent):
        super(CarControlWidget, self).__init__(parent=parent)

        self.initUi()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Left:
            print('Left Arrow Pressed')
        elif key == Qt.Key_Right:
            print('Right Arrow Pressed')
        elif key == Qt.Key_Up:
            print('Up Arrow Pressed')
        elif key == Qt.Key_Down:
            print('Down Arrow Pressed')

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_Left:
            print('Left Arrow Released')
        elif key == Qt.Key_Right:
            print('Right Arrow Released')
        elif key == Qt.Key_Up:
            print('Up Arrow Released')
        elif key == Qt.Key_Down:
            print('Down Arrow Released')

    def mouseMoveEvent(self, event):
        pos = event.pos()

        print "x: %d - y: %d" % (event.x(), event.y())

    def initUi(self):
        self.setFixedHeight(200)
        self.setFixedWidth(200)
        # self.setMouseTracking(True)
        self.grabKeyboard()

    def shutdown(self):
        self.releaseKeyboard()


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








