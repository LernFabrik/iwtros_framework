#!/usr/bin/python

import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *


def window():
    app = QApplication(sys.argv)
    win = QDialog()
    b1 = QPushButton(win)
    b1.setText("LAUNCH")
    b1.move(50,20)
    b1.clicked.connect(b1_clicked)

    b2 = QPushButton(win)
    b2.setText("CONNECT")
    b2.move(50,50)
    QObject.connect(b2, SIGNAL("clicked()"), b2_clicked)

    win.setGeometry(100,100,200,100)
    win.setWindowTitle("ROS Framework")
    win.show()
    sys.exit(app.exec_())


def b1_clicked():
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Critical)

    msg.setText("This is a message box")
    msg.setInformativeText("This is a additional message")
    msg.setWindowTitle("message box demo")
    msg.setDetailedText("The detain are follows:")
    msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
    msg.buttonClicked.connect(msgbtn)

    retval = msg.exec_()
    print "value of pressed message box button:", retval


def msgbtn(i):
   print "Button pressed is:",i.text()


def b2_clicked():
    print "Connect button is clicked"


if __name__ == '__main__':
    window()