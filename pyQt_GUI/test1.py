import sys
from PyQt4.QtGui import *

app = QApplication(sys.argv)
button = QPushButton("Hello World", None)
button.show()
app.exec_()