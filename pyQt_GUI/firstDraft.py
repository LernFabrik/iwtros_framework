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

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(450, 324)
        self.launchGroup = QtGui.QGroupBox(Dialog)
        self.launchGroup.setGeometry(QtCore.QRect(10, 10, 371, 63))
        self.launchGroup.setObjectName(_fromUtf8("launchGroup"))
        self.gridLayout = QtGui.QGridLayout(self.launchGroup)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.launchEnv = QtGui.QPushButton(self.launchGroup)
        self.launchEnv.setObjectName(_fromUtf8("launchEnv"))
        self.gridLayout.addWidget(self.launchEnv, 0, 0, 1, 1)
        self.enabledSimulation = QtGui.QCheckBox(self.launchGroup)
        self.enabledSimulation.setObjectName(_fromUtf8("enabledSimulation"))
        self.gridLayout.addWidget(self.enabledSimulation, 0, 1, 1, 1)
        self.enabledRviz = QtGui.QCheckBox(self.launchGroup)
        self.enabledRviz.setObjectName(_fromUtf8("enabledRviz"))
        self.gridLayout.addWidget(self.enabledRviz, 0, 2, 1, 1)
        self.enabledGazebo = QtGui.QCheckBox(self.launchGroup)
        self.enabledGazebo.setObjectName(_fromUtf8("enabledGazebo"))
        self.gridLayout.addWidget(self.enabledGazebo, 0, 3, 1, 1)
        self.stopButton = QtGui.QPushButton(Dialog)
        self.stopButton.setGeometry(QtCore.QRect(90, 240, 99, 27))
        self.stopButton.setObjectName(_fromUtf8("stopButton"))
        self.ftsControlGroup = QtGui.QGroupBox(Dialog)
        self.ftsControlGroup.setGeometry(QtCore.QRect(20, 100, 331, 91))
        self.ftsControlGroup.setObjectName(_fromUtf8("ftsControlGroup"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.ftsControlGroup)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.ftsGo = QtGui.QPushButton(self.ftsControlGroup)
        self.ftsGo.setObjectName(_fromUtf8("ftsGo"))
        self.horizontalLayout.addWidget(self.ftsGo)
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
        self.ftsGo.raise_()
        self.selPanda.raise_()
        self.listPosition.raise_()

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "Dialog", None))
        self.launchGroup.setTitle(_translate("Dialog", "ROS Framework - Start", None))
        self.launchEnv.setText(_translate("Dialog", "LAUNCH", None))
        self.enabledSimulation.setText(_translate("Dialog", "Simulation", None))
        self.enabledRviz.setText(_translate("Dialog", "rviz", None))
        self.enabledGazebo.setText(_translate("Dialog", "Gazebo", None))
        self.stopButton.setText(_translate("Dialog", "STOP", None))
        self.ftsControlGroup.setTitle(_translate("Dialog", "FTS Control", None))
        self.ftsGo.setText(_translate("Dialog", "GO", None))
        self.selPanda.setText(_translate("Dialog", "PANDA", None))
        self.selIIWA.setText(_translate("Dialog", "IIWA 7", None))
        self.selUr5.setText(_translate("Dialog", "UR5", None))
        __sortingEnabled = self.listPosition.isSortingEnabled()
        self.listPosition.setSortingEnabled(False)
        item = self.listPosition.item(0)
        item.setText(_translate("Dialog", "Position 1", None))
        item = self.listPosition.item(1)
        item.setText(_translate("Dialog", "Position 2", None))
        item = self.listPosition.item(2)
        item.setText(_translate("Dialog", "Position 3", None))
        item = self.listPosition.item(3)
        item.setText(_translate("Dialog", "Position 4", None))
        item = self.listPosition.item(4)
        item.setText(_translate("Dialog", "Position 5", None))
        self.listPosition.setSortingEnabled(__sortingEnabled)


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Dialog = QtGui.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

