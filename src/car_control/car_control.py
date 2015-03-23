# Copyright (c) 2014, Linh Nguyen <nvl1109@gmail.com>
# All rights reserved.

from __future__ import division
import os
import rospkg
import threading
import time

import rospy
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget
from rqt_gui_py.plugin import Plugin

from pi_dc_motor.msg import Motor


class CarControlWidget(QWidget):
    def __init__(self, parent):
        super(CarControlWidget, self).__init__(parent=parent)

        self._leftDir = 0
        self._rightDir = 0

        self._pushThread = None
        self._isStop = False

        self._upKey = False
        self._downKey = False
        self._leftKey = False
        self._rightKey = False

        self.initUi()
        self.initCar()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Left:
            rospy.logdebug('Left Arrow Pressed')
            self._leftKey = True
        elif key == Qt.Key_Right:
            rospy.logdebug('Right Arrow Pressed')
            self._rightKey = True
        elif key == Qt.Key_Up:
            rospy.logdebug('Up Arrow Pressed')
            self._upKey = True
        elif key == Qt.Key_Down:
            rospy.logdebug('Down Arrow Pressed')
            self._downKey = True

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_Left:
            rospy.logdebug('Left Arrow Released')
            self._leftKey = False
        elif key == Qt.Key_Right:
            rospy.logdebug('Right Arrow Released')
            self._rightKey = False
        elif key == Qt.Key_Up:
            rospy.logdebug('Up Arrow Released')
            self._upKey = False
        elif key == Qt.Key_Down:
            rospy.logdebug('Down Arrow Released')
            self._downKey = False

    def mouseMoveEvent(self, event):
        pos = event.pos()

        rospy.logdebug("x: %d - y: %d" % (event.x(), event.y()))

    def initUi(self):
        self.setFixedHeight(100)
        self.setFixedWidth(100)
        # self.setMouseTracking(True)
        self.grabKeyboard()
        rospy.loginfo("CarControl: keyboard grabed. Start control your car.")

    def initCar(self):
        self._publisher = rospy.Publisher('pi_motor_control', Motor, queue_size=10)
        # rospy.init_node('pi_car', anonymous=True)
        # Create push message thread
        self._pushThread = threading.Thread(target=self._pushMsgExec)
        self._pushThread.start()

    def processKeys(self):
        # Process the key status then update motor direction
        if (self._upKey and self._downKey):
            self._leftDir = 0
            self._rightDir = 0
            return
        if (self._leftKey and self._rightKey):
            self._leftDir = 0
            self._rightDir = 0
            return
        if self._upKey:
            if self._leftKey:
                self._leftDir = 0
                self._rightDir = 100
            elif self._rightKey:
                self._leftDir = 100
                self._rightDir = 0
            else:
                self._leftDir = 100
                self._rightDir = 100
        elif self._downKey:
            if self._leftKey:
                self._leftDir = 0
                self._rightDir = -100
            elif self._rightKey:
                self._leftDir = -100
                self._rightDir = 0
            else:
                self._leftDir = -100
                self._rightDir = -100
        else:
            if self._leftKey:
                self._leftDir = -100
                self._rightDir = 100
            elif self._rightKey:
                self._leftDir = 100
                self._rightDir = -100
            else:
                self._leftDir = 0
                self._rightDir = 0


    def _pushMsgExec(self):
        prevLeft = 0
        prevRight = 0
        while not self._isStop:
            self.processKeys()
            if not (prevLeft == prevRight == self._leftDir == self._rightDir == 0):
                # If has control signal
                m = Motor()
                m.left = self._leftDir
                m.right = self._rightDir
                if self._isStop:
                    m.left = 0
                    m.right = 0
                self._publisher.publish(m)
                prevLeft = self._leftDir
                prevRight = self._rightDir
                rospy.loginfo("CarControl: send message left:%d right:%d" % (self._leftDir, self._rightDir))
            time.sleep(0.1)

    def shutdown(self):
        self.releaseKeyboard()
        rospy.loginfo("CarControl: shutting down")
        self._isStop = True
        if self._pushThread:
            self._pushThread.join()
        rospy.loginfo("CarControl: shutdown DONE")


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








