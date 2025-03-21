""" demonstration_gui_elements_implementation.py Implements the user interface elements (frontend)

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1223, 950)
        font = QtGui.QFont()
        font.setPointSize(11)
        MainWindow.setFont(font)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("QWidget#centralwidget {\n"
"    background-color: black;\n"
"}\n"
"\n"
"QWidget {\n"
"    background-color: white;\n"
"}\n"
"\n"
"QTextEdit {\n"
"    background-color: white;\n"
"}\n"
"\n"
"QRadioButton {\n"
"    color: white;\n"
"    background-color: black;\n"
"}\n"
"\n"
"QRadioButton::indicator {\n"
"    background-color: white;\n"
"    border: 1px solid gray;\n"
"    width: 15px;\n"
"    height: 15px;\n"
"    border-radius: 8px;\n"
"}\n"
"\n"
"QRadioButton::indicator:checked {\n"
"    background-color: lightgreen;\n"
"    border: 1px solid lightgreen;\n"
"}\n"
"\n"
"QCheckBox {\n"
"    color: white; /* Text color */\n"
"    background-color: black;\n"
"}\n"
"\n"
"QCheckBox::indicator {\n"
"    background-color: white; /* Default background when unchecked */\n"
"    border: 1px solid gray; /* Default border */\n"
"    width: 15px; /* Size of the checkbox */\n"
"    height: 15px; /* Size of the checkbox */\n"
"}\n"
"\n"
"QCheckBox::indicator:checked {\n"
"    background-color: lightgreen;\n"
"    border: 1px solid lightgreen;\n"
"}\n"
"\n"
"QListWidget {\n"
"    background-color: white;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: white;\n"
"    qproperty-alignment: AlignCenter;\n"
"    background-color: black;\n"
"}\n"
"\n"
"QPushButton {\n"
"    background-color: white;\n"
"    border-radius: 10px;\n"
"    color: black;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #e0e0e0;\n"
"}\n"
"\n"
"QLineEdit {\n"
"    background-color: white;\n"
"    color: black;\n"
"}\n"
"")
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(160, 40, 291, 51))
        font = QtGui.QFont()
        font.setPointSize(22)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.mediaPipeFeed = QtWidgets.QLabel(self.centralwidget)
        self.mediaPipeFeed.setGeometry(QtCore.QRect(560, 20, 640, 360))
        font = QtGui.QFont()
        font.setPointSize(30)
        self.mediaPipeFeed.setFont(font)
        self.mediaPipeFeed.setObjectName("mediaPipeFeed")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(10, 130, 81, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(250, 130, 55, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(60, 190, 121, 20))
        self.label_5.setStyleSheet("color:lightgreen")
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(64, 350, 61, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.label_6.setStyleSheet("color:lightgreen")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(70, 625, 55, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_7.setStyleSheet("color:lightgreen")
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setGeometry(QtCore.QRect(170, 510, 141, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.label_8.setStyleSheet("color:lightgreen")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(364, 510, 61, 16))
        self.label_9.setStyleSheet("color:lightgreen")
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(260, 190, 221, 20))
        self.label_10.setStyleSheet("color:lightgreen")
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.label_12 = QtWidgets.QLabel(self.centralwidget)
        self.label_12.setGeometry(QtCore.QRect(550, 770, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_12.setFont(font)
        self.label_12.setObjectName("label_12")
        self.clearLogsButton = QtWidgets.QPushButton(self.centralwidget)
        self.clearLogsButton.setGeometry(QtCore.QRect(1110, 770, 93, 28))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.clearLogsButton.setFont(font)
        self.clearLogsButton.setObjectName("clearLogsButton")
        self.refreshDemoListButton = QtWidgets.QPushButton(self.centralwidget)
        self.refreshDemoListButton.setGeometry(QtCore.QRect(430, 630, 20, 20))
        self.refreshDemoListButton.setIconSize(self.refreshDemoListButton.size()) 
        self.refreshDemoListButton.setObjectName("refreshDemoListButton")
        self.refreshDemoListButton.setStyleSheet("""
            QPushButton {
                border: none;
                padding: 0;
                background: transparent;
            }
        """)
        self.deleteDemoButton = QtWidgets.QPushButton(self.centralwidget)
        self.deleteDemoButton.setGeometry(QtCore.QRect(470, 630, 20, 20))
        self.deleteDemoButton.setIconSize(self.deleteDemoButton.size()) 
        self.deleteDemoButton.setObjectName("deleteDemoButton")
        self.deleteDemoButton.setStyleSheet("""
            QPushButton {
                border: none;
                padding: 0;
                background: transparent;
            }
        """)
        self.connectButton = QtWidgets.QPushButton(self.centralwidget)
        self.connectButton.setGeometry(QtCore.QRect(400, 120, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.connectButton.setFont(font)
        self.connectButton.setObjectName("connectButton")
        self.pepperLogo = QtWidgets.QLabel(self.centralwidget)
        self.pepperLogo.setGeometry(QtCore.QRect(20, 20, 121, 91))
        font = QtGui.QFont()
        font.setPointSize(22)
        self.pepperLogo.setFont(font)
        self.pepperLogo.setText("")
        self.pepperLogo.setObjectName("pepperLogo")
        self.robotIP = QtWidgets.QLineEdit(self.centralwidget)
        self.robotIP.setGeometry(QtCore.QRect(100, 130, 151, 22))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.robotIP.setFont(font)
        self.robotIP.setObjectName("robotIP")
        self.port = QtWidgets.QLineEdit(self.centralwidget)
        self.port.setGeometry(QtCore.QRect(310, 130, 71, 22))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.port.setFont(font)
        self.port.setObjectName("port")
        self.demoName = QtWidgets.QLineEdit(self.centralwidget)
        self.demoName.setGeometry(QtCore.QRect(30, 550, 121, 22))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.demoName.setFont(font)
        self.demoName.setObjectName("demoName")
        self.startDemonstrateButton = QtWidgets.QPushButton(self.centralwidget)
        self.startDemonstrateButton.setGeometry(QtCore.QRect(70, 230, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.startDemonstrateButton.setFont(font)
        self.startDemonstrateButton.setObjectName("startDemonstrateButton")
        self.stopDemonstrateButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopDemonstrateButton.setGeometry(QtCore.QRect(70, 280, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.stopDemonstrateButton.setFont(font)
        self.stopDemonstrateButton.setObjectName("stopDemonstrateButton")
        self.inputMapDemonstrate1 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapDemonstrate1.setGeometry(QtCore.QRect(260, 240, 221, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.inputMapDemonstrate1.setFont(font)
        self.inputMapDemonstrate1.setText("")
        self.inputMapDemonstrate1.setObjectName("inputMapDemonstrate1")
        self.inputMapDemonstrate2 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapDemonstrate2.setGeometry(QtCore.QRect(260, 290, 221, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.inputMapDemonstrate2.setFont(font)
        self.inputMapDemonstrate2.setText("")
        self.inputMapDemonstrate2.setObjectName("inputMapDemonstrate2")
        self.stopRecordButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopRecordButton.setGeometry(QtCore.QRect(70, 430, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.stopRecordButton.setFont(font)
        self.stopRecordButton.setObjectName("stopRecordButton")
        self.startRecordButton = QtWidgets.QPushButton(self.centralwidget)
        self.startRecordButton.setGeometry(QtCore.QRect(70, 380, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.startRecordButton.setFont(font)
        self.startRecordButton.setObjectName("startRecordButton")
        self.inputMapRecord1 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapRecord1.setGeometry(QtCore.QRect(260, 450, 221, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.inputMapRecord1.setFont(font)
        self.inputMapRecord1.setText("")
        self.inputMapRecord1.setObjectName("inputMapRecord1")
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setGeometry(QtCore.QRect(260, 350, 221, 20))
        self.label_11.setStyleSheet("color:lightgreen")
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_11.setFont(font)
        self.label_11.setObjectName("label_11")
        self.inputMapRecord2 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapRecord2.setGeometry(QtCore.QRect(260, 400, 221, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.inputMapRecord2.setFont(font)
        self.inputMapRecord2.setText("")
        self.inputMapRecord2.setObjectName("inputMapRecord2")
        self.stopReplayButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopReplayButton.setGeometry(QtCore.QRect(70, 720, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.stopReplayButton.setFont(font)
        self.stopReplayButton.setObjectName("stopReplayButton")
        self.startReplayButton = QtWidgets.QPushButton(self.centralwidget)
        self.startReplayButton.setGeometry(QtCore.QRect(70, 670, 101, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.startReplayButton.setFont(font)
        self.startReplayButton.setObjectName("startReplayButton")
        self.systemLogsBox = QtWidgets.QTextBrowser(self.centralwidget)
        self.systemLogsBox.setGeometry(QtCore.QRect(550, 400, 651, 361))
        self.systemLogsBox.setObjectName("systemLogsBox")
        self.demonstrateIcon = QtWidgets.QLabel(self.centralwidget)
        self.demonstrateIcon.setGeometry(QtCore.QRect(190, 180, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.demonstrateIcon.setFont(font)
        self.demonstrateIcon.setText("")
        self.demonstrateIcon.setObjectName("demonstrateIcon")
        self.recordIcon = QtWidgets.QLabel(self.centralwidget)
        self.recordIcon.setGeometry(QtCore.QRect(190, 340, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.recordIcon.setFont(font)
        self.recordIcon.setText("")
        self.recordIcon.setObjectName("recordIcon")
        self.connectedIcon = QtWidgets.QLabel(self.centralwidget)
        self.connectedIcon.setGeometry(QtCore.QRect(510, 120, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.connectedIcon.setFont(font)
        self.connectedIcon.setText("")
        self.connectedIcon.setObjectName("connectedIcon")
        self.biomoRadio = QtWidgets.QRadioButton(self.centralwidget)
        self.biomoRadio.setGeometry(QtCore.QRect(370, 530, 170, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.biomoRadio.setFont(font)
        self.biomoRadio.setObjectName("biomoRadio")
        self.butterworthRadio = QtWidgets.QRadioButton(self.centralwidget)
        self.butterworthRadio.setGeometry(QtCore.QRect(370, 560, 170, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.butterworthRadio.setFont(font)
        self.butterworthRadio.setObjectName("butterworthRadio")
        self.noFilterRadio = QtWidgets.QRadioButton(self.centralwidget)
        self.noFilterRadio.setGeometry(QtCore.QRect(370, 590, 170, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.noFilterRadio.setFont(font)
        self.noFilterRadio.setObjectName("noFilterRadio")
        self.estimatedAnglesCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.estimatedAnglesCheck.setGeometry(QtCore.QRect(180, 550, 170, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.estimatedAnglesCheck.setFont(font)
        self.estimatedAnglesCheck.setObjectName("estimatedAnglesCheck")
        self.measuredAnglesCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.measuredAnglesCheck.setGeometry(QtCore.QRect(180, 580, 170, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.measuredAnglesCheck.setFont(font)
        self.measuredAnglesCheck.setObjectName("measuredAnglesCheck")
        self.label_13 = QtWidgets.QLabel(self.centralwidget)
        self.label_13.setGeometry(QtCore.QRect(270, 630, 131, 21))
        self.label_13.setStyleSheet("color:lightgreen")
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_13.setFont(font)
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(self.centralwidget)
        self.label_14.setGeometry(QtCore.QRect(10, 510, 141, 20))
        self.label_14.setStyleSheet("color:lightgreen")
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_14.setFont(font)
        self.label_14.setObjectName("label_14")
        self.demoList = QtWidgets.QListWidget(self.centralwidget)
        self.demoList.setGeometry(QtCore.QRect(270, 660, 221, 201))
        self.demoList.setObjectName("demoList")
        item = QtWidgets.QListWidgetItem()
        self.demoList.addItem(item)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1223, 31))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "PepperTrace Tool"))
        self.mediaPipeFeed.setText(_translate("MainWindow", "Media Pipe Feed "))
        self.label_3.setText(_translate("MainWindow", "Robot IP"))
        self.label_4.setText(_translate("MainWindow", "Port"))
        self.label_5.setText(_translate("MainWindow", "Demonstrate"))
        self.label_6.setText(_translate("MainWindow", "Record"))
        self.label_7.setText(_translate("MainWindow", "Replay"))
        self.label_8.setText(_translate("MainWindow", "Data To Record"))
        self.label_9.setText(_translate("MainWindow", "Filter"))
        self.label_10.setText(_translate("MainWindow", "User-defined Input Map"))
        self.label_12.setText(_translate("MainWindow", "System Logs"))
        self.clearLogsButton.setText(_translate("MainWindow", "CLEAR"))
        self.connectButton.setText(_translate("MainWindow", "CONNECT"))
        self.robotIP.setText(_translate("MainWindow", ""))
        self.port.setText(_translate("MainWindow", "5995"))
        self.startDemonstrateButton.setText(_translate("MainWindow", "START"))
        self.stopDemonstrateButton.setText(_translate("MainWindow", "STOP"))
        self.stopRecordButton.setText(_translate("MainWindow", "STOP"))
        self.startRecordButton.setText(_translate("MainWindow", "START"))
        self.label_11.setText(_translate("MainWindow", "User-defined Input Map"))
        self.stopReplayButton.setText(_translate("MainWindow", "STOP"))
        self.startReplayButton.setText(_translate("MainWindow", "START"))
        self.biomoRadio.setText(_translate("MainWindow", "Biological Motion"))
        self.butterworthRadio.setText(_translate("MainWindow", "Butterworth Filter"))
        self.noFilterRadio.setText(_translate("MainWindow", "No Filter"))
        self.estimatedAnglesCheck.setText(_translate("MainWindow", "Estimated Angles"))
        self.measuredAnglesCheck.setText(_translate("MainWindow", "Measured Angles"))
        self.estimatedAnglesCheck.setChecked(True)
        self.measuredAnglesCheck.setChecked(True)
        self.biomoRadio.setChecked(True)
        self.label_13.setText(_translate("MainWindow", "Demonstrations"))
        self.label_14.setText(_translate("MainWindow", "Demo Name"))
        __sortingEnabled = self.demoList.isSortingEnabled()

