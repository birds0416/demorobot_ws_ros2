import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import subprocess

class MainGui(QWidget):
    def __init__(self):
        super().__init__()
        self.win = QWidget()
        self.win.setFixedSize(640, 480)
        self.initUI()
    
    def initUI(self):
        self.main_layout = QGridLayout()

        location_btn1 = QPushButton("위치 1")
        location_btn1.clicked.connect(lambda: self.btn_click(1))
        self.main_layout.addWidget(location_btn1, 0, 0)

        location_btn2 = QPushButton("위치 2")
        location_btn2.clicked.connect(lambda: self.btn_click(2))
        self.main_layout.addWidget(location_btn2, 1, 0)
        
        location_btn3 = QPushButton("위치 3")
        location_btn3.clicked.connect(lambda: self.btn_click(3))
        self.main_layout.addWidget(location_btn3, 2, 0)

        location_btn4 = QPushButton("위치 4")
        location_btn4.clicked.connect(lambda: self.btn_click(4))
        self.main_layout.addWidget(location_btn4, 0, 1)

        location_btn5 = QPushButton("위치 5")
        location_btn5.clicked.connect(lambda: self.btn_click(5))
        self.main_layout.addWidget(location_btn5, 1, 1)

        location_btn6 = QPushButton("위치 6")
        location_btn6.clicked.connect(lambda: self.btn_click(6))
        self.main_layout.addWidget(location_btn6, 2, 1)

        self.win.setLayout(self.main_layout)
        self.win.show()
    
    def btn_click(self, num):
        pass
    
    def emerUI(self):
        self.emer_layout = QGridLayout()



        self.win.setLayout(self.emer_layout)
        self.win.show()

        


# def btn_click():
#     subprocess.run(["ros2", "run", "demo_nodes_cpp", "talker"])

# def btn_click():
#     clicked_btn = sender()
#     if clicked_btn ==location_btn1:
#         pass