import os
import time
import threading

from ament_index_python.packages import get_package_share_directory
from qt_gui.plugin import Plugin
from pathlib import Path
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QWindow
from python_qt_binding.QtCore import Qt
from qt_gui.settings import Settings
from shell_cmd import sh
import subprocess

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

class EmbedPlotjuggler(Plugin):
    """
    This plugin allows to embed a Qt plotjuggler into a rqt plugin.
    """

    def __init__(self, context):
        super(EmbedPlotjuggler, self).__init__(context)
        self.setObjectName('EmbedPlotjuggler')
        self._command = ''
        self._window_name = 'PlotJuggler'
        self._window_id = None
        self._external_window_widget = None
        self._process = None
        self._timeout_to_plotjuggler_discovery = 20.0

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(
            get_package_share_directory("rqt_plotjuggler"),
            "resource",
            "embed_plotjuggler.ui",
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("EmbedPlotjugglerUi")
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.context = context

        self._widget.grap_plotjuggler_window_pushButton.pressed.connect(self.trigger_configuration)
        self._widget.start_plotjuggler_pushButton.pressed.connect(self.start_plotjuggler_in_thread)
        self._widget.start_plotjuggler_with_layout_pushButton.pressed.connect(self.start_plotjuggler_in_thread_with_layout)
        self._widget.reload_layouts_pushButton.pressed.connect(self.reload_plotjuggler_config_file_list)

    def grab_window(self):
        self._widget.doubleSpinBox_lower_body_primary.setValue((float)(random.uniform(0, 1.8)))
        self._widget.doubleSpinBox_upper_body_primary.setValue((float)(random.uniform(-2.5, 0.0)))

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("package_name", self._widget.package_name_lineEdit.text())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.package_name_lineEdit.setText(instance_settings.value('package_name'))
        self.reload_plotjuggler_config_file_list()

    def reload_plotjuggler_config_file_list(self):
        package_name = self._widget.package_name_lineEdit.text()
        try:
            self._package_dir = get_package_share_directory(package_name)
        except KeyError:
            print(f"Package '{package_name}' not found.")
            return None

        self._config_folder_path = os.path.join(
            self._package_dir,
            'config',
            'ui',
        )

        self._widget.layouts_comboBox.clear()
        try:
            for config in Path(self._config_folder_path).glob('*plotjuggler*.xml'):
                self._widget.layouts_comboBox.addItem(config.name)
        except FileNotFoundError:
            print(f"The directory {self._config_folder_path} does not exist.")
        except PermissionError:
            print(f"Permission denied to access {self._config_folder_path}.")

    def start_plotjuggler_in_thread(self):
        self._widget.start_plotjuggler_pushButton.hide()
        self._widget.start_plotjuggler_with_layout_pushButton.hide()
        # Create a new thread to run the subprocess
        thread = threading.Thread(target=self.start_plotjuggler)
        thread.start()

    def start_plotjuggler(self):
        subprocess.run(f'ros2 run plotjuggler plotjuggler', shell=True, capture_output=True, text=True)
        return None

    def start_plotjuggler_in_thread_with_layout(self):
        package_name = self._widget.package_name_lineEdit.text()
        try:
            self._package_dir = get_package_share_directory(package_name)
        except KeyError:
            print(f"Package '{package_name}' not found.")
            return None

        self._widget.start_plotjuggler_pushButton.hide()
        self._widget.layout_selector_groupBox.hide()
        # Create a new thread to run the subprocess
        thread = threading.Thread(target=self.start_plotjuggler_with_layout)
        thread.start()

    def start_plotjuggler_with_layout(self):
        current_file_selected = self._widget.layouts_comboBox.currentText()
        layout_file = os.path.join(
            self._package_dir,
            "config", "ui",
            current_file_selected,
        )
        subprocess.run(f'ros2 run plotjuggler plotjuggler --layout {layout_file}', shell=True, capture_output=True, text=True)
        return None

    def trigger_configuration(self):
        self._widget.grap_plotjuggler_window_pushButton.hide()
        # Refresh plugin!
        if self._external_window_widget is not None:
            self._widget.verticalLayout.removeWidget(self._external_window_widget)
        
        self.add_external_window_widget()

    def add_external_window_widget(self):
        # The command is prepended with exec so it becomes the shell executing it
        self.wait_for_window_id()
        print("plotjuggler id: " + str(self._window_id))
        if self._window_id is not None:
            # window_id = 0x02a00106
            # Create a the window that will contain the program
            plotjuggler = QWindow.fromWinId(int(self._window_id, 16))
            # FramelessWindowHint is necessary for the window to effectively get embedded
            plotjuggler.setFlags(Qt.FramelessWindowHint)
            widget = QWidget.createWindowContainer(plotjuggler)

            # Store it for later
            self._external_window_widget = widget

            # Set all margins and spacing to 0 to maximize plotjuggler usage
            self._widget.verticalLayout.setContentsMargins(0, 0, 0, 0)
            self._widget.verticalLayout.setSpacing(0)

            self._widget.verticalLayout.addWidget(self._external_window_widget)

            # Give the title (for rqt_gui compisitions) some information
            if self.context.serial_number() < 2:
                self._widget.setWindowTitle('{}      ({})'.format(self._widget.windowTitle(), self._window_name))
            else:
                self._widget.setWindowTitle('{} ({}) ({})'.format(self._widget.windowTitle(),
                                                                self.context.serial_number(), self._window_name))

    def wait_for_window_id(self, timeout=2.0):
        """
        Keep trying to find the Window ID for a PID until we find it or we timeout.
        """
        ini_t = time.time()
        now = time.time()
        while (now - ini_t) < timeout and self._window_id is None:
            self._window_id = self.get_window_id_by_window_name()
            print(self._window_id)
            time.sleep(0.5)
            now = time.time()
        if self._window_id is None:
            print("DID NOT FIND plotjuggler WINDOW")
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
                    print("Found plotjuggler")
                    return fields[0]
        return None
