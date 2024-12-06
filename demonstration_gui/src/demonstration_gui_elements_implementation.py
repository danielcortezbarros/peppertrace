""" demonstration_gui_elements_implementation.py Definition of visual elements of GUI

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
        MainWindow.resize(1301, 915)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setKerning(True)
        MainWindow.setFont(font)
        MainWindow.setStyleSheet("background:black")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("QWidget {\n"
"     background-color:black;\n"
"}\n"
"\n"
"QTextEdit {\n"
"    background-color: white;\n"
"}\n"
"\n"
"QRadioButton {\n"
"    color: white;\n"
"}\n"
"QRadioButton::indicator {\n"
"    background-color: white; \n"
"    border: 1px solid gray;  \n"
"    width: 15px; \n"
"    height: 15px; \n"
"    border-radius: 8px; \n"
"}\n"
"QRadioButton::indicator:checked {\n"
"    background-color: lightgreen;\n"
"    border: 1px solid lightgreen; \n"
"}\n"
"\n"
"QCheckBox {\n"
"    color: white; /* Text color */\n"
"}\n"
"\n"
"QCheckBox::indicator {\n"
"    background-color: white; /* Default background when unchecked */\n"
"    border: 1px solid gray;  /* Default border */\n"
"    width: 15px;             /* Size of the checkbox */\n"
"    height: 15px;            /* Size of the checkbox */\n"
"}\n"
"\n"
"QCheckBox::indicator:checked {\n"
"    background-color: lightgreen;\n"
"    border: 1px solid lightgreen;\n"
"}\n"
"\n"
"\n"
"QListWidget {\n"
"    background-color: white;\n"
"}\n"
"\n"
"QLabel {\n"
"color: white;\n"
"qproperty-alignment: AlignCenter;\n"
"}\n"
"\n"
"QPushButton {\n"
"background-color:white;\n"
"border-radius: 10px;\n"
"color:black;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"background-color:#e0e0e0;\n"
"}\n"
"")
        self.centralwidget.setObjectName("centralwidget")
        self.systemLogsBox = QtWidgets.QTextBrowser(self.centralwidget)
        self.systemLogsBox.setGeometry(QtCore.QRect(610, 420, 640, 371))
        font.setPointSize(12)
        self.systemLogsBox.setFont(font)
        self.systemLogsBox.setObjectName("systemLogsBox")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(60, 490, 81, 21))
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setStyleSheet("color:lightgreen;")
        self.label_2.setObjectName("label_2")
        self.startRecordButton = QtWidgets.QPushButton(self.centralwidget)
        self.startRecordButton.setGeometry(QtCore.QRect(60, 530, 131, 41))
        font.setPointSize(11)
        self.startRecordButton.setFont(font)
        self.startRecordButton.setObjectName("startRecordButton")
        self.stopRecordButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopRecordButton.setGeometry(QtCore.QRect(60, 580, 131, 41))
        self.stopRecordButton.setFont(font)
        self.stopRecordButton.setObjectName("stopRecordButton")
        self.pepperLogo = QtWidgets.QLabel(self.centralwidget)
        self.pepperLogo.setGeometry(QtCore.QRect(30, 40, 131, 91))
        self.pepperLogo.setText("")
        self.pepperLogo.setPixmap(QtGui.QPixmap("../../../Pictures/peppergui/Pepper.png"))
        self.pepperLogo.setScaledContents(True)
        self.pepperLogo.setObjectName("pepperLogo")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(170, 70, 380, 41))
        font.setPointSize(25)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.clearLogsButton = QtWidgets.QPushButton(self.centralwidget)
        self.clearLogsButton.setGeometry(QtCore.QRect(1140, 800, 111, 31))
        font.setPointSize(11)
        self.clearLogsButton.setFont(font)
        self.clearLogsButton.setObjectName("clearLogsButton")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(610, 800, 121, 21))
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setGeometry(QtCore.QRect(30, 150, 81, 21))
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(260, 490, 230, 21))
        self.label_6.setFont(font)
        self.label_6.setStyleSheet("color:lightgreen;")
        self.label_6.setObjectName("label_6")
        self.connectButton = QtWidgets.QPushButton(self.centralwidget)
        self.connectButton.setGeometry(QtCore.QRect(390, 140, 131, 41))
        font.setPointSize(11)
        self.connectButton.setFont(font)
        self.connectButton.setObjectName("connectButton")
        self.robotIP = QtWidgets.QLineEdit(self.centralwidget)
        self.robotIP.setGeometry(QtCore.QRect(122, 150, 131, 25))
        font.setPointSize(12)
        self.robotIP.setFont(font)
        self.robotIP.setStyleSheet("background:white")
        self.robotIP.setObjectName("robotIP")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(260, 150, 41, 21))
        font.setPointSize(14)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.port = QtWidgets.QLineEdit(self.centralwidget)
        self.port.setGeometry(QtCore.QRect(310, 150, 61, 25))
        font.setPointSize(12)
        self.port.setFont(font)
        self.port.setStyleSheet("background:white")
        self.port.setObjectName("port")
        self.inputMapRecord1 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapRecord1.setGeometry(QtCore.QRect(260, 540, 211, 21))
        font.setPointSize(14)
        self.inputMapRecord1.setFont(font)
        self.inputMapRecord1.setText("")
        self.inputMapRecord1.setAlignment(QtCore.Qt.AlignCenter)
        self.inputMapRecord1.setObjectName("inputMapRecord1")
        self.inputMapRecord2 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapRecord2.setGeometry(QtCore.QRect(260, 590, 211, 21))
        self.inputMapRecord2.setFont(font)
        self.inputMapRecord2.setText("")
        self.inputMapRecord2.setAlignment(QtCore.Qt.AlignCenter)
        self.inputMapRecord2.setObjectName("inputMapRecord2")
        self.connectedIcon = QtWidgets.QLabel(self.centralwidget)
        self.connectedIcon.setGeometry(QtCore.QRect(530, 140, 41, 41))
        self.connectedIcon.setText("")
        self.connectedIcon.setPixmap(QtGui.QPixmap("../../../Pictures/peppergui/red.png"))
        self.connectedIcon.setScaledContents(True)
        self.connectedIcon.setObjectName("connectedIcon")
        self.startDemonstrateButton = QtWidgets.QPushButton(self.centralwidget)
        self.startDemonstrateButton.setGeometry(QtCore.QRect(60, 360, 131, 41))
        font.setPointSize(11)
        self.startDemonstrateButton.setFont(font)
        self.startDemonstrateButton.setObjectName("startDemonstrateButton")
        self.stopDemonstrateButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopDemonstrateButton.setGeometry(QtCore.QRect(60, 410, 131, 41))
        self.stopDemonstrateButton.setFont(font)
        self.stopDemonstrateButton.setObjectName("stopDemonstrateButton")
        self.inputMapDemonstrate1 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapDemonstrate1.setGeometry(QtCore.QRect(260, 370, 211, 21))
        font.setPointSize(14)
        self.inputMapDemonstrate1.setFont(font)
        self.inputMapDemonstrate1.setText("")
        self.inputMapDemonstrate1.setAlignment(QtCore.Qt.AlignCenter)
        self.inputMapDemonstrate1.setObjectName("inputMapDemonstrate1")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(260, 320, 230, 21))
        self.label_7.setFont(font)
        self.label_7.setStyleSheet("color:lightgreen;")
        self.label_7.setObjectName("label_7")
        self.inputMapDemonstrate2 = QtWidgets.QLabel(self.centralwidget)
        self.inputMapDemonstrate2.setGeometry(QtCore.QRect(260, 420, 211, 21))
        self.inputMapDemonstrate2.setFont(font)
        self.inputMapDemonstrate2.setText("")
        self.inputMapDemonstrate2.setAlignment(QtCore.Qt.AlignCenter)
        self.inputMapDemonstrate2.setObjectName("inputMapDemonstrate2")
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(60, 320, 120, 21))
        self.label_10.setFont(font)
        self.label_10.setStyleSheet("color:lightgreen;")
        self.label_10.setObjectName("label_10")
        self.demonstrateIcon = QtWidgets.QLabel(self.centralwidget)
        self.demonstrateIcon.setGeometry(QtCore.QRect(200, 310, 41, 41))
        self.demonstrateIcon.setText("")
        self.demonstrateIcon.setPixmap(QtGui.QPixmap("../../../Pictures/peppergui/red.png"))
        self.demonstrateIcon.setScaledContents(True)
        self.demonstrateIcon.setObjectName("demonstrateIcon")
        self.recordIcon = QtWidgets.QLabel(self.centralwidget)
        self.recordIcon.setGeometry(QtCore.QRect(200, 480, 41, 41))
        self.recordIcon.setText("")
        self.recordIcon.setPixmap(QtGui.QPixmap("../../../Pictures/peppergui/red.png"))
        self.recordIcon.setScaledContents(True)
        self.recordIcon.setObjectName("recordIcon")
        self.mediaPipeFeed = QtWidgets.QLabel(self.centralwidget)
        self.mediaPipeFeed.setGeometry(QtCore.QRect(609, 30, 640, 360))
        font.setPointSize(40)
        self.mediaPipeFeed.setFont(font)
        self.mediaPipeFeed.setStyleSheet("border: 1px solid white;")
        self.mediaPipeFeed.setObjectName("mediaPipeFeed")
        self.butterworthRadio = QtWidgets.QRadioButton(self.centralwidget)
        self.butterworthRadio.setGeometry(QtCore.QRect(300, 230, 195, 23))
        font.setPointSize(14)
        self.butterworthRadio.setFont(font)
        self.butterworthRadio.setObjectName("butterworthRadio")
        self.biomoRadio = QtWidgets.QRadioButton(self.centralwidget)
        self.biomoRadio.setGeometry(QtCore.QRect(100, 230, 195, 23))
        self.biomoRadio.setChecked(True)
        self.biomoRadio.setFont(font)
        self.biomoRadio.setObjectName("biomoRadio")
        self.estimatedAnglesCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.estimatedAnglesCheck.setGeometry(QtCore.QRect(60, 650, 195, 21))
        self.estimatedAnglesCheck.setFont(font)
        self.estimatedAnglesCheck.setObjectName("estimatedAnglesCheck")
        self.measuredAnglesCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.measuredAnglesCheck.setGeometry(QtCore.QRect(60, 690, 195, 21))
        self.measuredAnglesCheck.setFont(font)
        self.measuredAnglesCheck.setObjectName("measuredAnglesCheck")
        self.imagesCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.imagesCheck.setGeometry(QtCore.QRect(60, 730, 195, 21))
        self.imagesCheck.setFont(font)
        self.imagesCheck.setObjectName("imagesCheck")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1301, 22))
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
        self.label_2.setText(_translate("MainWindow", "Record"))
        self.startRecordButton.setText(_translate("MainWindow", "START"))
        self.stopRecordButton.setText(_translate("MainWindow", "STOP"))
        self.label_4.setText(_translate("MainWindow", "PepperTrace Recorder"))
        self.clearLogsButton.setText(_translate("MainWindow", "CLEAR"))
        self.label_5.setText(_translate("MainWindow", "System Logs"))
        self.label_8.setText(_translate("MainWindow", "Robot IP"))
        self.label_6.setText(_translate("MainWindow", "User-defined Input Map"))
        self.connectButton.setText(_translate("MainWindow", "CONNECT"))
        self.robotIP.setText(_translate("MainWindow", "172.29.111.230"))
        self.label_9.setText(_translate("MainWindow", "Port"))
        self.port.setText(_translate("MainWindow", "5995"))
        self.startDemonstrateButton.setText(_translate("MainWindow", "START"))
        self.stopDemonstrateButton.setText(_translate("MainWindow", "STOP"))
        self.label_7.setText(_translate("MainWindow", "User-defined Input Map"))
        self.label_10.setText(_translate("MainWindow", "Demonstrate"))
        self.mediaPipeFeed.setText(_translate("MainWindow", "MediaPipe Feed"))
        self.butterworthRadio.setText(_translate("MainWindow", "Butterworth Filter"))
        self.biomoRadio.setText(_translate("MainWindow", "Biological Motion"))
        self.estimatedAnglesCheck.setText(_translate("MainWindow", "Estimated angles"))
        self.measuredAnglesCheck.setText(_translate("MainWindow", "Measured angles"))
        self.imagesCheck.setText(_translate("MainWindow", "Images"))
