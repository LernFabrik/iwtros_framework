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
        self.ftsGoButton = QtGui.QPushButton(self.ftsControlGroup)
        self.ftsGoButton.setObjectName(_fromUtf8("ftsGoButton"))
        self.horizontalLayout.addWidget(self.ftsGoButton)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.selPanda = QtGui.QRadioButton(self.ftsControlGroup)
        self.selPanda.setObjectName(_fromUtf8("selPanda"))
        self.verticalLayout.addWidget(self.selPanda)
        self.selIIWA = QtGui.QRadioButton(self.ftsControlGroup)
        self.selIIWA.setObjectName(_fromUtf8("selIIWA"))
        self.verticalLayout.addWidget(self.selIIWA)
        self.selUr5 = QtGui.QRadioButton(self.ftsControlGroup)
        self.selUr5.setObjectName(_fromUtf8("selUr5"))
        self.verticalLayout.addWidget(self.selUr5)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.listPosition = QtGui.QListWidget(self.ftsControlGroup)
        self.listPosition.setObjectName(_fromUtf8("listPosition"))
        item = QtGui.QListWidgetItem()
        self.listPosition.addItem(item)
        item = QtGui.QListWidgetItem()
        self.listPosition.addItem(item)
        item = QtGui.QListWidgetItem()
        self.listPosition.addItem(item)
        item = QtGui.QListWidgetItem()
        self.listPosition.addItem(item)
        item = QtGui.QListWidgetItem()
        self.listPosition.addItem(item)
        self.horizontalLayout.addWidget(self.listPosition)
        self.ftsGoButton.raise_()
        self.selPanda.raise_()
        self.listPosition.raise_()

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
        self.ftsGoButton.setText(_translate("rosFrame", "GO", None))
        self.selPanda.setText(_translate("rosFrame", "PANDA", None))
        self.selIIWA.setText(_translate("rosFrame", "IIWA 7", None))
        self.selUr5.setText(_translate("rosFrame", "UR5", None))
        __sortingEnabled = self.listPosition.isSortingEnabled()
        self.listPosition.setSortingEnabled(False)
        item = self.listPosition.item(0)
        item.setText(_translate("rosFrame", "Position 1", None))
        item = self.listPosition.item(1)
        item.setText(_translate("rosFrame", "Position 2", None))
        item = self.listPosition.item(2)
        item.setText(_translate("rosFrame", "Position 3", None))
        item = self.listPosition.item(3)
        item.setText(_translate("rosFrame", "Position 4", None))
        item = self.listPosition.item(4)
        item.setText(_translate("rosFrame", "Position 5", None))
        self.listPosition.setSortingEnabled(__sortingEnabled)
    
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
        os.system('killall -9 roscore &')
        os.system('killall -9 rosmaster &')
        self.launchEnvButton.setDisabled(False)
        


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    rosFrame = QtGui.QDialog()
    ui = Ui_rosFrame(rosFrame)
    ui.setupUi(rosFrame)
    rosFrame.show()
    sys.exit(app.exec_())

