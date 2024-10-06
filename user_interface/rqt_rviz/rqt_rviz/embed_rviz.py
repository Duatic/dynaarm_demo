import os
import time

from ament_index_python.packages import get_package_share_directory
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QWindow
from python_qt_binding.QtCore import Qt
from qt_gui.settings import Settings
from shell_cmd import sh
import subprocess

class EmbedRviz(Plugin):
    """
    This plugin allows to embed a Qt rviz into a rqt plugin.
    """

    def __init__(self, context):
        super(EmbedRviz, self).__init__(context)
        self.setObjectName('EmbedRviz')
        self._command = ''
        self._window_name = 'RViz'
        self._window_id = None
        self._external_window_widget = None
        self._process = None
        self._timeout_to_rviz_discovery = 20.0

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(
            get_package_share_directory("rqt_rviz"),
            "resource",
            "embed_rviz.ui",
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("EmbedRvizUi")
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.context = context

        # self.trigger_configuration()

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        self.trigger_configuration()

    def trigger_configuration(self):
        # Refresh plugin!
        if self._external_window_widget is not None:
            self._widget.verticalLayout.removeWidget(self._external_window_widget)
        
        self.add_external_window_widget()

    def add_external_window_widget(self):
        # The command is prepended with exec so it becomes the shell executing it
        self.wait_for_window_id()
        print("rviz id: " + str(self._window_id))
        if self._window_id is not None:
            # window_id = 0x02a00106
            # Create a the window that will contain the program
            rviz = QWindow.fromWinId(int(self._window_id, 16))
            # FramelessWindowHint is necessary for the window to effectively get embedded
            rviz.setFlags(Qt.FramelessWindowHint)
            widget = QWidget.createWindowContainer(rviz)

            # Store it for later
            self._external_window_widget = widget

            # Set all margins and spacing to 0 to maximize rviz usage
            self._widget.verticalLayout.setContentsMargins(0, 0, 0, 0)
            self._widget.verticalLayout.setSpacing(0)

            self._widget.verticalLayout.addWidget(self._external_window_widget)

            # Give the title (for rqt_gui compisitions) some information
            if self.context.serial_number() < 2:
                self._widget.setWindowTitle('{}      ({})'.format(self._widget.windowTitle(), self._window_name))
            else:
                self._widget.setWindowTitle('{} ({}) ({})'.format(self._widget.windowTitle(),
                                                                self.context.serial_number(), self._window_name))

    def wait_for_window_id(self, timeout=10.0):
        """
        Keep trying to find the Window ID for a PID until we find it or we timeout.
        """
        print('wait_for_window_id')
        ini_t = time.time()
        now = time.time()
        while (now - ini_t) < timeout and self._window_id is None:
            self._window_id = self.get_window_id_by_window_name()

            time.sleep(0.5)
            now = time.time()
        if self._window_id is None:
            print("DID NOT FIND RVIZ WINDOW")
        return

    def get_window_id_by_window_name(self):
        """
        Get the window ID based on the name of the window, None if not found.
        Uses wmctrl and parses it output to find it
        """
        # Looks like:
        # 0x03c00041  0 3498   skipper Mozilla Firefox
        # WindowID    ? PID       USER   Window Name

        result = subprocess.run('wmctrl -lp', shell=True, capture_output=True, text=True)
        # Find the line with the PID we are looking for
        for line in result.stdout.splitlines():
            fields = line.split()
            for field in fields:
                if self._window_name in field:
                    print("Found RViz")
                    return fields[0]
        return None
