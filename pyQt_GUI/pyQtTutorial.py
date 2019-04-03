import sys
from PyQt4 import QtGui as qt

def window():
    app = qt.QGuiApplication(sys.argv)
    w = qt.QWidget()
    b = qt.QLabel(w)
    b.setText("Hello world")
    



if __name__ == '__main__':
    window()