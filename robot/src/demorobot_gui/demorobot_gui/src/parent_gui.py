from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import sys

import main_gui as mg
import emer_gui as eg

class ParentGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle('구난로봇 메뉴')
        
        self.central_widget = QStackedWidget()
        self.setCentralWidget(self.central_widget)

        self.main = mg.MainWindow()
        self.emer = eg.EmerWindow()

        self.central_widget.addWidget(self.main)
        self.central_widget.addWidget(self.emer)

        self.central_widget.setCurrentWidget(self.main)
    
    def switch_to_main(self):
        self.central_widget.setCurrentWidget(self.main)
    
    def switch_to_emer(self):
        self.central_widget.setCurrentWidget(self.emer)

        