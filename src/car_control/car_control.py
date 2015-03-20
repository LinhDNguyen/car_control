# Copyright (c) 2014, Linh Nguyen <nvl1109@gmail.com>
# All rights reserved.

from __future__ import division
import os
import rospkg

import rospy
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget
from rqt_gui_py.plugin import Plugin


class CarControl(Plugin):


    def __init__(self, context):
        super(CarControl, self).__init__(context)
        self.setObjectName('CarControl')

        self._publisher = None

        self._widget = QWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle("Car Control" + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)


    @Slot(str)
    def _on_topic_changed(self, topic):
        pass

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass








