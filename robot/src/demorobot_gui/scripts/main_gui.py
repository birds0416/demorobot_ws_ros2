import sys
from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import subprocess

msgFont = QFont('NanumGothic', 15)
msgFont.setBold(True)

class UIWindow(object):
    def setupUI(self, MainWindow):
        # MainWindow.setGeometry(50, 50, 400, 450)
        MainWindow.setFixedSize(640, 480)
        MainWindow.setWindowTitle("위치 안내")
        self.centralwidget = QWidget(MainWindow)
        
        self.location_select_msg = QLabel('안내받을 목적지를 선택하세요.', self.centralwidget)
        self.location_select_msg.setFont(msgFont)
        self.location_select_msg.setAlignment(Qt.AlignCenter)
        self.location_select_msg.setGeometry(170, 60, 300, 100)
        
        self.location_btn1 = QPushButton("위치 1", self.centralwidget)
        self.location_btn1.setGeometry(100, 150, 200, 50)
        
        self.location_btn2 = QPushButton("위치 2", self.centralwidget)
        self.location_btn2.setGeometry(100, 250, 200, 50)
        
        self.location_btn3 = QPushButton("위치 3", self.centralwidget)
        self.location_btn3.setGeometry(100, 350, 200, 50)
        
        self.location_btn4 = QPushButton("위치 4", self.centralwidget)
        self.location_btn4.setGeometry(340, 150, 200, 50)
        
        self.location_btn5 = QPushButton("위치 5", self.centralwidget)
        self.location_btn5.setGeometry(340, 250, 200, 50)
        
        self.location_btn6 = QPushButton("위치 6", self.centralwidget)
        self.location_btn6.setGeometry(340, 350, 200, 50)
        
        MainWindow.setCentralWidget(self.centralwidget)


class UIToolTab(object):
    def setupUI(self, MainWindow, num):
        # MainWindow.setGeometry(50, 50, 400, 450)
        MainWindow.setFixedSize(640, 480)
        MainWindow.setWindowTitle("위치 안내 메시지")
        self.centralwidget = QWidget(MainWindow)
        
        self.location_msg = QLabel('위치 {}로\n이동중입니다'.format(num), self.centralwidget)
        self.location_msg.setFont(msgFont)
        self.location_msg.setAlignment(Qt.AlignCenter)
        self.location_msg.setGeometry(220, 170, 200, 100)
        
        self.backBTN = QPushButton("돌아가기", self.centralwidget)
        self.backBTN.move(100, 350)
        MainWindow.setCentralWidget(self.centralwidget)

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.uiWindow = UIWindow()
        self.uiToolTab = UIToolTab()
        
        self.bat_percent = 100
        self.cpu_temp = 10
        self.bat_stat = QLabel('배터리상태: {}%'.format(str(self.bat_percent)), self)
        self.bat_stat.setFont(QFont('Arial', 12))
        self.bat_stat.setGeometry(135, 10, 200, 30)
        self.cpu_stat = QLabel('CPU: {}℃'.format(str(self.cpu_temp)), self)
        self.cpu_stat.setFont(QFont('Arial', 12))
        self.cpu_stat.setGeometry(400, 10, 200, 30)
        
        self.startUIWindow()

    def startUIToolTab(self, num):
        self.uiToolTab.setupUI(self, num)
        self.uiToolTab.backBTN.clicked.connect(self.startUIWindow)
        self.show()

    def startUIWindow(self):
        self.uiWindow.setupUI(self)
        self.uiWindow.location_btn1.clicked.connect(lambda: self.startUIToolTab(1))
        self.uiWindow.location_btn2.clicked.connect(lambda: self.startUIToolTab(2))
        self.uiWindow.location_btn3.clicked.connect(lambda: self.startUIToolTab(3))
        self.uiWindow.location_btn4.clicked.connect(lambda: self.startUIToolTab(4))
        self.uiWindow.location_btn5.clicked.connect(lambda: self.startUIToolTab(5))
        self.uiWindow.location_btn6.clicked.connect(lambda: self.startUIToolTab(6))
        self.show()
        
    def btn3(self):
        print("Number 3 pressed!!!!!!")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    sys.exit(app.exec_())