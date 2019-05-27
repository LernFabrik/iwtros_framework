# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'firstDraft.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

import os
import sys
import signal
import subprocess
from PyQt4 import QtCore, QtGui
import rospy
from iwtros_msgs.msg import ftsControl

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_rosFrame(object):
    def __init__(self, rosFrame):
        self.enable_sim = True
        self.enabled_rviz = True
        self.pub = rospy.Publisher('startFtsOperation', ftsControl, queue_size=10)
        self.ftsmsg = ftsControl()
        

    def setupUi(self, rosFrame):
        rosFrame.setObjectName(_fromUtf8("rosFrame"))
        rosFrame.resize(450, 324)
        self.launchGroup = QtGui.QGroupBox(rosFrame)
        self.launchGroup.setGeometry(QtCore.QRect(10, 10, 371, 63))
        self.launchGroup.setObjectName(_fromUtf8("launchGroup"))
        self.gridLayout = QtGui.QGridLayout(self.launchGroup)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        # LAUNCH Button
        self.launchEnvButton = QtGui.QPushButton(self.launchGroup)
        self.launchEnvButton.setObjectName(_fromUtf8("launchEnvButton"))
        QtCore.QObject.connect(self.launchEnvButton, QtCore.SIGNAL("clicked()"), self.launchButtonCallback)
        self.gridLayout.addWidget(self.launchEnvButton, 0, 0, 1, 1)
        # Enabled Simulation checkbox
        self.enabledSimulation = QtGui.QCheckBox(self.launchGroup)
        self.enabledSimulation.setObjectName(_fromUtf8("enabledSimulation"))
        self.enabledSimulation.setChecked(True)
        self.enabledSimulation.stateChanged.connect(lambda:self.checkBoxCallback(self.enabledSimulation))
        self.gridLayout.addWidget(self.enabledSimulation, 0, 1, 1, 1)
        # Enable rviz
        self.enabledRviz = QtGui.QCheckBox(self.launchGroup)
        self.enabledRviz.setObjectName(_fromUtf8("enabledRviz"))
        self.enabledRviz.setChecked(True)
        self.enabledRviz.stateChanged.connect(lambda:self.checkBoxCallback(self.enabledRviz))
        self.gridLayout.addWidget(self.enabledRviz, 0, 2, 1, 1)
        # STOP Button
        self.stopButton = QtGui.QPushButton(rosFrame)
        self.stopButton.setGeometry(QtCore.QRect(160, 240, 99, 27))
        QtCore.QObject.connect(self.stopButton, QtCore.SIGNAL("clicked()"), self.stopButtonCallback)
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.ftsControlGroup = QtGui.QGroupBox(rosFrame)
        self.ftsControlGroup.setGeometry(QtCore.QRect(20, 100, 331, 91))
        self.ftsControlGroup.setObjectName(_fromUtf8("ftsControlGroup"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.ftsControlGroup)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.selPanda = QtGui.QRadioButton(self.ftsControlGroup)
        # SELECT ROBOT
        ## PANDA
        self.selPanda.setObjectName(_fromUtf8("selPanda"))
        self.selPanda.setChecked(False)
        self.selPanda.toggled.connect(lambda:self.seletionCallback(self.selPanda))
        self.verticalLayout.addWidget(self.selPanda)
        ## IIWA
        self.selIIWA = QtGui.QRadioButton(self.ftsControlGroup)
        self.selIIWA.setObjectName(_fromUtf8("selIIWA"))
        self.selIIWA.setChecked(False)
        self.selIIWA.toggled.connect(lambda:self.seletionCallback(self.selIIWA))
        self.verticalLayout.addWidget(self.selIIWA)
        ## UR5
        self.selUr5 = QtGui.QRadioButton(self.ftsControlGroup)
        self.selUr5.setObjectName(_fromUtf8("selUr5"))
        self.selUr5.setChecked(False)
        self.selUr5.toggled.connect(lambda:self.seletionCallback(self.selUr5))
        self.verticalLayout.addWidget(self.selUr5)
        #List of positions
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.Pose1 = QtGui.QCheckBox(self.ftsControlGroup)
        self.Pose1.setObjectName(_fromUtf8("Pose1"))
        self.verticalLayout_2.addWidget(self.Pose1)
        self.Pose1.stateChanged.connect(lambda:self.poseCallback(self.Pose1))
        self.Pose2 = QtGui.QCheckBox(self.ftsControlGroup)
        self.Pose2.setObjectName(_fromUtf8("Pose2"))
        self.verticalLayout_2.addWidget(self.Pose2)
        self.Pose2.stateChanged.connect(lambda:self.poseCallback(self.Pose2))
        self.Pose3 = QtGui.QCheckBox(self.ftsControlGroup)
        self.Pose3.setObjectName(_fromUtf8("Pose3"))
        self.verticalLayout_2.addWidget(self.Pose3)
        self.Pose3.stateChanged.connect(lambda:self.poseCallback(self.Pose3))
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.ftsGoButton = QtGui.QPushButton(self.ftsControlGroup)
        self.ftsGoButton.setObjectName(_fromUtf8("ftsGoButton"))
        QtCore.QObject.connect(self.ftsGoButton, QtCore.SIGNAL("clicked()"), self.ftsGoCallback)
        self.horizontalLayout.addWidget(self.ftsGoButton)
        self.ftsGoButton.raise_()


        self.retranslateUi(rosFrame)
        QtCore.QMetaObject.connectSlotsByName(rosFrame)

    def retranslateUi(self, rosFrame):
        rosFrame.setWindowTitle(_translate("rosFrame", "ROS Framework", None))
        self.launchGroup.setTitle(_translate("rosFrame", "ROS Framework - Start", None))
        self.launchEnvButton.setText(_translate("rosFrame", "LAUNCH", None))
        self.enabledSimulation.setText(_translate("rosFrame", "Simulation", None))
        self.enabledRviz.setText(_translate("rosFrame", "rviz", None))
        self.stopButton.setText(_translate("rosFrame", "STOP", None))
        self.ftsControlGroup.setTitle(_translate("rosFrame", "FTS Control", None))
        self.selPanda.setText(_translate("rosFrame", "PANDA", None))
        self.selIIWA.setText(_translate("rosFrame", "IIWA 7", None))
        self.selUr5.setText(_translate("rosFrame", "UR5", None))
        self.Pose1.setText(_translate("rosFrame", "Pose 1", None))
        self.Pose2.setText(_translate("rosFrame", "Pose 2", None))
        self.Pose3.setText(_translate("rosFrame", "Pose 3", None))
        self.ftsGoButton.setText(_translate("rosFrame", "GO", None))
    
    def checkBoxCallback(self, checked):
        print "--------- Check Box State ---------"
        if checked.text() == "Simulation":
            if checked.isChecked() == False:
                self.enable_sim = False
                print checked.text() + " is disabled!"
            else:
                self.enable_sim = True
                print checked.text() + " is enabled!"
        if checked.text() == "rviz":
            if checked.isChecked() == False:
                self.enabled_rviz = False
                print checked.text() + " is disabled!"
            else:
                self.enabled_rviz = True
                print checked.text() + " is enabled!"
    
    def seletionCallback(self, toggled):
        print " radio button selection"
        if toggled.text() == "PANDA":
            print "......Panda selection....."
            self.selIIWA.setChecked(False)
            self.selUr5.setChecked(False)
            self.ftsmsg.selRobot = 2
        if toggled.text() == "IIWA 7":
            print "......iiwa selection....."
            self.selPanda.setChecked(False)
            self.selUr5.setChecked(False)
            self.ftsmsg.selRobot = 0
        if toggled.text() == "UR5":
            print "......ur5 selection....."
            self.selPanda.setChecked(False)
            self.selIIWA.setChecked(False)
            self.ftsmsg.selRobot = 1
 
    def launchButtonCallback(self):
        if self.enable_sim == False:
            if self.enabled_rviz == False:
                print "launching sim:=false rviz:=false"
                os.system('roslaunch iwtros_launch iwtros_env.launch sim:=false rviz:=false &')
            else:
                print "launching sim:=false"
                os.system('roslaunch iwtros_launch iwtros_env.launch sim:=false &')
        else:
            if self.enabled_rviz == False:
                print "launching rviz:=false"
                os.system('roslaunch iwtros_launch iwtros_env.launch rviz:=false &')
            else:
                print "launching"
                os.system('roslaunch iwtros_launch iwtros_env.launch &')
        self.launchEnvButton.setDisabled(True)
    
    def stopButtonCallback(self):
        print sys.argv
        os.system('rosnode kill -a &')
        #os.system('killall -9 roscore &')
        os.system('killall -9 rosmaster &')
        self.launchEnvButton.setDisabled(False)
        """
        TODO: Complete wait message box 
        """
        msg = QtGui.QMessageBox()
        msg.setIcon(QtGui.QMessageBox.Critical)
        msg.setText("Killing ROS Master")
        msg.setWindowTitle("Wait!")
        msg.setDetailedText("wait until ROS nodes are killed and you see the message done in the terminal")
        msg.setStandardButtons(QtGui.QMessageBox.Ok | QtGui.QMessageBox.Cancel)
    
    def poseCallback(self, checked):
        print "list is selected"
        self.ftsmsg.pose.orientation.z = 0
        self.ftsmsg.pose.orientation.w = 1
        self.ftsmsg.pose.position.y = -3
        if checked.text() == "Pose 1":
            self.Pose2.setChecked(False)
            self.Pose3.setChecked(False)
            self.ftsmsg.pose.position.x = -7
        if checked.text() == "Pose 2":
            self.Pose1.setChecked(False)
            self.Pose3.setChecked(False)
            self.ftsmsg.pose.position.x = -4
        if checked.text() == "Pose 3":
            self.Pose1.setChecked(False)
            self.Pose2.setChecked(False)
            self.ftsmsg.pose.position.x = -1
    
    def ftsGoCallback(self):
        self.pub.publish(self.ftsmsg)

if __name__ == "__main__":
    rospy.init_node('guiControl', anonymous=False)
    app = QtGui.QApplication(sys.argv)
    rosFrame = QtGui.QDialog()
    ui = Ui_rosFrame(rosFrame)
    ui.setupUi(rosFrame)
    rosFrame.show()
    sys.exit(app.exec_())

