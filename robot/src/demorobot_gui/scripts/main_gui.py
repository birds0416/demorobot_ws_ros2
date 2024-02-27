from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

import sys
import subprocess
import cv2
import numpy as np

msgFont = QFont('NanumGothic', 15)
msgFont.setBold(True)

class NavigationWindow(object):
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

class NavigationTab(object):
    def setupUI(self, MainWindow, num):
        # MainWindow.setGeometry(50, 50, 400, 450)
        MainWindow.setFixedSize(640, 480)
        MainWindow.setWindowTitle("위치 안내 메시지")
        self.centralwidget = QWidget(MainWindow)

        if num == 1:
            MainWindow.navigation_coordinate = (10, 10)
        elif num == 2:
            MainWindow.navigation_coordinate = (20, 20)
        elif num == 3:
            MainWindow.navigation_coordinate = (30, 30)
        elif num == 4:
            MainWindow.navigation_coordinate = (40, 40)
        elif num == 5:
            MainWindow.navigation_coordinate = (50, 50)
        elif num == 6:
            MainWindow.navigation_coordinate = (60, 60)
        
        self.location_msg = QLabel('위치 {}로\n이동중입니다'.format(num), self.centralwidget)
        self.location_msg.setFont(msgFont)
        self.location_msg.setAlignment(Qt.AlignCenter)
        self.location_msg.setGeometry(220, 170, 200, 100)
        
        # self.location_msg = QLabel('위치 {}에\n도착하였습니다'.format(num), self.centralwidget)
        # self.location_msg.setFont(msgFont)
        # self.location_msg.setAlignment(Qt.AlignCenter)
        # self.location_msg.setGeometry(220, 170, 200, 100)
        
        self.backBTN = QPushButton("돌아가기", self.centralwidget)
        self.backBTN.move(100, 350)
        MainWindow.setCentralWidget(self.centralwidget)

class ChargingWindow(object):
    def setupUI(self, MainWindow):
        MainWindow.setFixedSize(640, 480)
        MainWindow.setWindowTitle("충전복귀모드")
        self.centralwidget = QWidget(MainWindow)
        
        self.location_select_msg = QLabel('배터리 충전을 위해\n복귀중입니다.', self.centralwidget)
        self.location_select_msg.setFont(msgFont)
        self.location_select_msg.setAlignment(Qt.AlignCenter)
        self.location_select_msg.setGeometry(170, 170, 300, 100)
        
        MainWindow.setCentralWidget(self.centralwidget)

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.installEventFilter(self)
        
        self.navigationWindow = NavigationWindow()
        self.navigationTab = NavigationTab()
        self.chargingWindow = ChargingWindow()
        
        self.bat_percent = 100
        self.cpu_temp = 10
        
        self.bat_stat = QLabel('배터리상태: {}%'.format(str(self.bat_percent)), self)
        self.bat_stat.setFont(QFont('NanumGothic', 12))
        self.bat_stat.setGeometry(135, 10, 200, 30)
        self.cpu_stat = QLabel('CPU: {}℃'.format(str(self.cpu_temp)), self)
        self.cpu_stat.setFont(QFont('NanumGothic', 12))
        self.cpu_stat.setGeometry(400, 10, 200, 30)

        self.navigation_coordinate = (0, 0)
        
        if self.bat_percent <= 30:
            self.init_charging_Window()
        else:
            self.init_navigation_Window()
    
    def return_navigation_coord(self):
        return self.navigation_coordinate
            
    def init_navigation_Tab(self, num):
        self.navigationTab.setupUI(self, num)
        self.navigationTab.backBTN.clicked.connect(self.init_navigation_Window)
        self.show()

    def init_navigation_Window(self):
        self.navigationWindow.setupUI(self)
        self.navigationWindow.location_btn1.clicked.connect(lambda: self.init_navigation_Tab(1))
        self.navigationWindow.location_btn2.clicked.connect(lambda: self.init_navigation_Tab(2))
        self.navigationWindow.location_btn3.clicked.connect(lambda: self.init_navigation_Tab(3))
        self.navigationWindow.location_btn4.clicked.connect(lambda: self.init_navigation_Tab(4))
        self.navigationWindow.location_btn5.clicked.connect(lambda: self.init_navigation_Tab(5))
        self.navigationWindow.location_btn6.clicked.connect(lambda: self.init_navigation_Tab(6))
        self.show()
        
    def init_charging_Window(self):
        self.chargingWindow.setupUI(self)
        self.show()

# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     w = MainWindow()
#     sys.exit(app.exec_())
