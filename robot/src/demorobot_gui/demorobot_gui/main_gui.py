import sys
from PyQt5.QtWidgets import *

def main(args=None):
    app = QApplication(sys.argv)
    win = QWidget()
    win.setFixedSize(640, 480)

    lineedit = QLineEdit()

    win.show()
    app.exec_()

if __name__ == '__main__':
    main()