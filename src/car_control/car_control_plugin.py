# Author: Linh Nguyen <nvl1109@gmail.com>

from rqt_py_common.plugin_container_widget import PluginContainerWidget
from rqt_gui_py.plugin import Plugin

from car_control.car_control_widget import CarControlWidget


class CarControlPlugin(Plugin):

    def __init__(self, plugin_context):
        super(CarControlPlugin, self).__init__(plugin_context)
        self._plugin_context = plugin_context

        self._car_control_widget = CarControlWidget(self, plugin_context)
        self.mainwidget = PluginContainerWidget(self._car_control_widget,
                                                 True, False)

        if self._plugin_context.serial_number() > 1:
            self.mainwidget.setWindowTitle(self.mainwidget.windowTitle() +
                                   (' (%d)' % plugin_context.serial_number()))

        plugin_context.add_widget(self.mainwidget)

    def get_widget(self):
        return self.mainwidget

    def shutdown_plugin(self):
        self.mainwidget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        self.mainwidget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self.mainwidget.restore_settings(plugin_settings, instance_settings)

    def _update_msg(self):
        """
        Update necessary components (per topic) regularly
        """
        self._moveit_widget.update_topic_table()
