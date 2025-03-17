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
from PyQt5.QtGui import QImage, QPixmap, QIcon
from demonstration_gui_elements_implementation import Ui_MainWindow
import json
from demonstration_gui_ros_interface_implementation import RosThread
import rospkg
import os
import numpy as np
import cv2
import rospy
import time
import re


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, gui_system_logs_topic, gui_commands_topic, skeletal_model_feed_topic, demo_data_dir, parent=None):
        """
        Class constructor. MainWindow implements the logic for handling incoming ROS messages and updating the GUI.

        Args:
            gui_system_logs_topic(str): Topic to receive system logs for display in the system logs box
            gui_commands_topic(str): Topic to send commands to the system 
            skeletal_model_feed_topic(str): Topic to receive images with skeletal model overlay for display
        """

        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.trying_connect = False
        self.connected = False
        self.demonstrate_running = False
        self.demo_data_dir = demo_data_dir

        # Get the path to the package
        package_path = rospkg.RosPack().get_path('programming_by_demonstration')

        # Construct the path to the config file relative to the package path
        self.gui_images_path = os.path.join(package_path, 'demonstration_gui', 'images')
        self.ui.pepperLogo.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'pepper.png')))
        self.ui.connectedIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
        self.ui.demonstrateIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
        self.ui.recordIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
        self.ui.refreshDemoListButton.setIcon(QIcon(os.path.join(self.gui_images_path, 'refresh.png')))
        self.ui.deleteDemoButton.setIcon(QIcon(os.path.join(self.gui_images_path, 'trash.png')))

        self.ros_thread = RosThread(topic_sub=gui_system_logs_topic, topic_pub=gui_commands_topic, skeletal_model_feed_topic=skeletal_model_feed_topic)
        self.ros_thread.start()
        self.ros_thread.update_gui_signal.connect(self.display_info)
        self.ros_thread.image_signal.connect(self.display_image_feed)
        self.connect_gui_buttons()
        self.update_demo_list()

    def connect_gui_buttons(self):
        """Connect GUI buttons to their callback functions."""

        self.ui.connectButton.clicked.connect(lambda: self.send_command('CONNECT'))
        self.ui.startDemonstrateButton.clicked.connect(lambda: self.send_command('START_DEMONSTRATE'))
        self.ui.stopDemonstrateButton.clicked.connect(lambda: self.send_command('STOP_DEMONSTRATE'))
        self.ui.startRecordButton.clicked.connect(lambda: self.send_command('START_RECORD'))
        self.ui.stopRecordButton.clicked.connect(lambda: self.send_command('STOP_RECORD'))
        self.ui.startReplayButton.clicked.connect(lambda: self.send_command('START_REPLAY'))
        self.ui.stopReplayButton.clicked.connect(lambda: self.send_command('STOP_REPLAY'))
        # self.ui.browseButton.clicked.connect(self.open_file_dialog)
        self.ui.clearLogsButton.clicked.connect(self.clear_systems_log_box)
        self.ui.refreshDemoListButton.clicked.connect(self.update_demo_list)
        self.ui.deleteDemoButton.clicked.connect(self.delete_demo)
        self.ui.biomoRadio.toggled.connect(self.update_filter)
        self.ui.butterworthRadio.toggled.connect(self.update_filter)
        self.ui.noFilterRadio.toggled.connect(self.update_filter)
        self.ui.estimatedAnglesCheck.stateChanged.connect(self.update_data_to_record)
        self.ui.measuredAnglesCheck.stateChanged.connect(self.update_data_to_record)

        self.ui.pepperLogo.setScaledContents(True)
        self.ui.connectedIcon.setScaledContents(True)
        self.ui.recordIcon.setScaledContents(True)
        self.ui.demonstrateIcon.setScaledContents(True)
       # self.ui.imagesCheck.stateChanged.connect(self.update_data_to_record)

    def send_command(self, cmd):
        """
        Send command to the system over the gui_commands_topic. 
        Depending on which button was pressed, check conditions, otherwise just forward the command.

        Args:
            cmd(str): command to send to the system. 
        """
        if cmd == 'CONNECT' and self.trying_connect is False:
            cmd= cmd + ',{},{}'.format(self.ui.robotIP.text(), self.ui.port.text())
            self.ros_thread.publish_signal.emit(cmd)
            self.trying_connect = True

        elif cmd == 'START_DEMONSTRATE':
            if self.connected == False:
                self.display_info("[WARNING] Please connect to the robot before demonstrating.")  
            else:
                self.ros_thread.publish_signal.emit(cmd)

        elif cmd =='START_RECORD':
            if self.ui.demoName.text() == '':
                self.display_info("[WARNING] Please set the Demo Name before recording.")    
                return
            # elif self.demonstrate_running == False:
            #     self.display_info("[WARNING] Please start demonstrate before recording.")
            else:
                cmd= cmd + ',{}'.format(self.ui.demoName.text())
                self.ros_thread.publish_signal.emit(cmd)
                
        elif cmd =='START_REPLAY':
            if self.demonstrate_running == True:
                self.display_info("[WARNING] Please stop demonstrating before replaying motion.")
                return
            else:
                selected_demo = self.get_selected_demo()
                #Check if not None
                if selected_demo is None:
                    self.display_info("[WARNING] Please select a demo before starting replay.")
                    return
                else:
                    cmd= cmd + ',{}'.format(selected_demo)
                    self.ros_thread.publish_signal.emit(cmd)
        else:
            self.ros_thread.publish_signal.emit(cmd)

    def get_selected_demo(self):
        """
        Get the currently selected demo from the QListWidget.

        Returns:
            str: The selected demo name, or None if nothing is selected.
        """
        selected_items = self.ui.demoList.selectedItems()
        if selected_items:
            return selected_items[0].text()
        else:
            rospy.logwarn("No demo selected.")
            return None

    def get_demo_list(self):
        """
        Gets a list of all demos in the demo_data folder, including subfolders.

        Returns:
            list: A sorted list of demo names with their file extensions removed.
        """
        demo_names = []
        for root, _, files in os.walk(self.demo_data_dir):
            for file in files:
                if file.endswith(".bag") and 'unit_test' not in file:
                    demo_name = os.path.splitext(file)[0]  # Remove the .bag extension
                    demo_names.append(demo_name)

        demo_names.sort()  # Sort the demo names alphabetically
        rospy.loginfo(f"Found demos: {demo_names}")
        return demo_names

    def update_demo_list(self):
        """Updates the QListWidget with the demo names from the demo_data folder."""

        demo_list = self.get_demo_list()
        self.ui.demoList.clear()
        self.ui.demoList.addItems(demo_list)
        

    def delete_demo(self):
        demo_to_delete = self.get_selected_demo()

        if demo_to_delete is None:
            self.display_info(f"[WARNING] Please select a demo to delete.")
            return

        print(demo_to_delete)

        # Remove digits from the demo name
        base_demo_name = re.sub(r'\d+$', '', demo_to_delete)
        file_path = os.path.join(self.demo_data_dir, base_demo_name, f"{demo_to_delete}.bag")
        print(file_path)

        # Check if the file exists before attempting to delete
        if not os.path.exists(file_path):
            self.display_info(f"[ERROR] Cannot delete: {file_path} does not exist.")
        else:
            os.remove(file_path)
            self.display_info(f"Deleted: {file_path}")

            # Check if the base_demo_name directory is empty
            base_demo_path = os.path.join(self.demo_data_dir, base_demo_name)
            if os.path.exists(base_demo_path) and not os.listdir(base_demo_path):  # Directory is empty
                os.rmdir(base_demo_path)  # Remove the empty directory
                self.display_info(f"Deleted empty directory: {base_demo_path}")

        # Refresh the demo list
        self.update_demo_list()


    def clear_systems_log_box(self):
        self.ui.systemLogsBox.clear()

    def open_file_dialog(self):
        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", "", "All Files (*.*)")
        if file_name:
            self.ui.replayLineEdit.setText(file_name)

    def update_filter(self):
        """ Sends signal to the system to change the filter applied to the skeletal model data."""

        sender = self.sender() 
        if sender.isChecked():
            if sender.text() == "Biological Motion":
                self.ros_thread.publish_signal.emit("FILTERbiological")
            elif sender.text() == "Butterworth Filter":
                self.ros_thread.publish_signal.emit("FILTERbutterworth")
            else:
                self.ros_thread.publish_signal.emit("FILTERno")


    def update_data_to_record(self):
        """Construct and publish what types of data to record based on checked checkboxes."""

        data_to_record = []
        if self.ui.estimatedAnglesCheck.isChecked():
            data_to_record.append("estimated_angles")
        if self.ui.measuredAnglesCheck.isChecked():
            data_to_record.append("measured_angles")
        # if self.ui.imagesCheck.isChecked():
        #     data_to_record.append("images")

        # Construct the command string
        command = f"RECORD{data_to_record}"

        # Publish the command to the ROS topic
        rospy.loginfo(f"Publishing command: {command}")
        self.ros_thread.publish_signal.emit(command)


    @QtCore.pyqtSlot(str)
    def display_info(self, info):
        """
        Function connected to a pyqtSignal, which forwards ROS messages on the gui_system_logs_topic
        Display system information in the system logs box. 
        Handle icon changes depending on the information to signal state changes to the user. 

        Args:
            info(str): information from the system
        """

        
        if info.startswith("Map"):

            # Display input map
            _, map_str = info.split(",", 1)
            map_dict = json.loads(map_str)

            self.ui.inputMapDemonstrate1.setText(map_dict["Demonstrate"]["Start"])
            self.ui.inputMapDemonstrate2.setText(map_dict["Demonstrate"]["Stop"])
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

            # Update GUI images
            if "ERROR" not in info:
                if "ROBOT CONNECTED" in info:
                    self.connected = True
                    self.ui.connectButton.setText("DISCONNECT")
                    self.ui.connectedIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'green.png')))
                    self.trying_connect = False
                elif "ROBOT DISCONNECTED" in info:
                    self.connected = False
                    self.ui.connectButton.setText("CONNECT")
                    self.ui.connectedIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
                elif "STOPPED DEMONSTRATING" in info:
                    self.demonstrate_running = False
                    self.ui.mediaPipeFeed.setPixmap(QPixmap())
                    self.ui.demonstrateIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
                elif "DEMONSTRATING" in info:
                    self.demonstrate_running = True
                    self.ui.demonstrateIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'green.png')))
                elif "STOPPED RECORDING" in info:
                    self.ui.recordIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
                    time.sleep(1)
                    self.update_demo_list()
                elif "RECORDING" in info:
                    self.ui.recordIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'green.png')))
               

            else:
                if "STOPPED DEMONSTRATING" in info:
                    self.ui.mediaPipeFeed.setPixmap(QPixmap())
                    self.demonstrate_running = False
                    self.ui.demonstrateIcon.setPixmap(QtGui.QPixmap(os.path.join(self.gui_images_path, 'red.png')))
                

    @QtCore.pyqtSlot(np.ndarray)
    def display_image_feed(self, cv_image):
        """
        Function connected to a pyqtSignal, which forwards ROS images on the skeletal_model_feed_topic for display.

        Args:
            cv_image(np.array): Image with skeletal model overlay
        """

        # Convert image from BGR to RGB
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Convert to QImage and QPixmap
        height, width, channel = cv_image_rgb.shape
        bytes_per_line = 3 * width
        qt_image = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)

        # Update QLabel
        self.ui.mediaPipeFeed.setPixmap(pixmap)

