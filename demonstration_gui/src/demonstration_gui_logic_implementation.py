""" demonstration_gui_logic_implementation.py Implements control logic for the GUI elements

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import QImage, QPixmap
from demonstration_gui_elements_implementation import Ui_MainWindow
import json
from demonstration_gui_ros_interface_implementation import RosThread
import rospkg
import os
import numpy as np
import cv2
import rospy


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, gui_system_logs_topic, gui_commands_topic, skeletal_model_feed_topic, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.teleop_running = False

        # Get the path to the package
        package_path = rospkg.RosPack().get_path('programming_from_demonstration')

        # Construct the path to the config file relative to the package path
        self.gui_images_path = os.path.join(package_path, 'demonstration_gui', 'images')
        self.ui.pepperLogo.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'pepper.png')))
        self.ui.connectedIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
        self.ui.teleopIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
        self.ui.recordIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))

        self.ros_thread = RosThread(topic_sub=gui_system_logs_topic, topic_pub=gui_commands_topic, skeletal_model_feed_topic=skeletal_model_feed_topic)
        self.ros_thread.start()
        self.ros_thread.update_gui_signal.connect(self.display_info)
        self.ros_thread.image_signal.connect(self.display_image_feed)
        self.connect_gui_buttons()

    def connect_gui_buttons(self):
        self.ui.connectButton.clicked.connect(lambda: self.send_command(f'CONNECT,{self.ui.robotIP.text()},{self.ui.port.text()}'))
        self.ui.startTeleopButton.clicked.connect(lambda: self.send_command('START_TELEOP'))
        self.ui.stopTeleopButton.clicked.connect(lambda: self.send_command('STOP_TELEOP'))
        self.ui.startRecordButton.clicked.connect(lambda: self.send_command('START_RECORD'))
        self.ui.stopRecordButton.clicked.connect(lambda: self.send_command('STOP_RECORD'))
        self.ui.clearLogsButton.clicked.connect(lambda: self.clear_systems_log_box())
        self.ui.biomoRadio.toggled.connect(self.update_filter)
        self.ui.butterworthRadio.toggled.connect(self.update_filter)
        self.ui.estimatedAnglesCheck.stateChanged.connect(self.update_data_to_record)
        self.ui.measuredAnglesCheck.stateChanged.connect(self.update_data_to_record)
        self.ui.imagesCheck.stateChanged.connect(self.update_data_to_record)

    def send_command(self, cmd):
        if cmd == 'START_RECORD' and self.teleop_running == False:
            self.display_info("[WARNING] Please start teleop before recording")
            return
        else:
            self.ros_thread.publish_signal.emit(cmd)

    def clear_systems_log_box(self):
        self.ui.systemLogsBox.clear()

    def update_filter(self):
        sender = self.sender() 
        if sender.isChecked():
            if sender.text() == "Biological Motion":
                self.ros_thread.publish_signal.emit("FILTERbiological")
            else:
                self.ros_thread.publish_signal.emit("FILTERbutterworth")

    def update_data_to_record(self):
        """Construct and publish the record command based on checked checkboxes."""
        topics = []
        if self.ui.estimatedAnglesCheck.isChecked():
            topics.append("estimated")
        if self.ui.measuredAnglesCheck.isChecked():
            topics.append("measured")
        if self.ui.imagesCheck.isChecked():
            topics.append("images")

        # Construct the command string
        command = f"RECORD{topics}"

        # Publish the command to the ROS topic
        rospy.loginfo(f"Publishing command: {command}")
        self.ros_thread.publish_signal.emit(command)


    @QtCore.pyqtSlot(str)
    def display_info(self, info):

        # display input map
        if info.startswith("Map"):

            _, map_str = info.split(",", 1)
            map_dict = json.loads(map_str)

            self.ui.inputMapTeleop1.setText(map_dict["Teleop"]["Start"])
            self.ui.inputMapTeleop2.setText(map_dict["Teleop"]["Stop"])
            self.ui.inputMapRecord1.setText(map_dict["Record"]["Start"])
            self.ui.inputMapRecord2.setText(map_dict["Record"]["Stop"])

        else: 
            # Log info to system logs box with color formatting
            text_color = "black" 
            if "ERROR" in info:
                text_color = "red"
            elif "WARNING" in info:
                text_color = "orange"

            formatted_text = f'<span style="color:{text_color};">{info}</span>'
            self.ui.systemLogsBox.append(formatted_text)
            self.ui.systemLogsBox.verticalScrollBar().setValue(self.ui.systemLogsBox.verticalScrollBar().maximum())

            # change GUI images
            if "ERROR" not in info:
                if "CONNECTED" in info:
                    self.ui.connectedIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'green.png')))
                elif "STOPPED TELEOPING" in info:
                    self.teleop_running = False
                    self.ui.mediaPipeFeed.setPixmap(QPixmap())
                    self.ui.teleopIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
                elif "TELEOPING" in info:
                    self.teleop_running = True
                    self.ui.teleopIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'green.png')))
                elif "STOPPED RECORDING" in info:
                    self.ui.recordIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
                elif "RECORDING" in info:
                    self.ui.recordIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'green.png')))

            else:
                if "STOPPED TELEOPING" in info:
                    self.ui.mediaPipeFeed.setPixmap(QPixmap())
                    self.teleop_running = False
                    self.ui.teleopIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))

    @QtCore.pyqtSlot(np.ndarray)
    def display_image_feed(self, cv_image):
        # Convert BGR to RGB
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Convert to QImage and QPixmap
        height, width, channel = cv_image_rgb.shape
        bytes_per_line = 3 * width
        qt_image = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)

        # Update QLabel
        self.ui.mediaPipeFeed.setPixmap(pixmap)

