# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'firstDraft.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

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
    def setupUi(self, rosFrame):
        rosFrame.setObjectName(_fromUtf8("rosFrame"))
        rosFrame.resize(450, 264)
        self.launchGroup = QtGui.QGroupBox(rosFrame)
        self.launchGroup.setGeometry(QtCore.QRect(10, 10, 371, 63))
        self.launchGroup.setObjectName(_fromUtf8("launchGroup"))
        self.gridLayout = QtGui.QGridLayout(self.launchGroup)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.launchEnvButton = QtGui.QPushButton(self.launchGroup)
        self.launchEnvButton.setObjectName(_fromUtf8("launchEnvButton"))
        self.gridLayout.addWidget(self.launchEnvButton, 0, 0, 1, 1)
        self.enabledSimulation = QtGui.QCheckBox(self.launchGroup)
        self.enabledSimulation.setObjectName(_fromUtf8("enabledSimulation"))
        self.gridLayout.addWidget(self.enabledSimulation, 0, 1, 1, 1)
        self.enabledRviz = QtGui.QCheckBox(self.launchGroup)
        self.enabledRviz.setObjectName(_fromUtf8("enabledRviz"))
        self.gridLayout.addWidget(self.enabledRviz, 0, 2, 1, 1)
        self.stopButton = QtGui.QPushButton(rosFrame)
        self.stopButton.setGeometry(QtCore.QRect(160, 210, 99, 27))
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.ftsControlGroup = QtGui.QGroupBox(rosFrame)
        self.ftsControlGroup.setGeometry(QtCore.QRect(20, 100, 331, 91))
        self.ftsControlGroup.setObjectName(_fromUtf8("ftsControlGroup"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.ftsControlGroup)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
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
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.Pose1 = QtGui.QCheckBox(self.ftsControlGroup)
        self.Pose1.setObjectName(_fromUtf8("Pose1"))
        self.verticalLayout_2.addWidget(self.Pose1)
        self.Pose2 = QtGui.QCheckBox(self.ftsControlGroup)
        self.Pose2.setObjectName(_fromUtf8("Pose2"))
        self.verticalLayout_2.addWidget(self.Pose2)
        self.Pose3 = QtGui.QCheckBox(self.ftsControlGroup)
        self.Pose3.setObjectName(_fromUtf8("Pose3"))
        self.verticalLayout_2.addWidget(self.Pose3)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.ftsGoButton = QtGui.QPushButton(self.ftsControlGroup)
        self.ftsGoButton.setObjectName(_fromUtf8("ftsGoButton"))
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


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    rosFrame = QtGui.QDialog()
    ui = Ui_rosFrame()
    ui.setupUi(rosFrame)
    rosFrame.show()
    sys.exit(app.exec_())

