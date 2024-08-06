from PyQt5.Qt import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

import sys
import subprocess
import cv2
import numpy as np

msgFont = QFont('NimbusMonoPS', 15)
msgFont.setBold(True)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("로봇 서버")
        self.setStyleSheet("background-color: white")
        self.installEventFilter(self)

        self.rgb_img_label = QLabel(self)
        self.rgb_img_label.setGeometry(10, 40, 640, 480)

        self.depth_img_label = QLabel(self)
        self.depth_img_label.setGeometry(660, 40, 640, 480)
    
    def show_rgb_img(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img)
        self.rgb_img_label.setPixmap(qt_img)
    
    def show_depth_img(self, cv_img):
        qt_img = self.convert_cv_qt(cv_img)
        self.depth_img_label.setPixmap(qt_img)
    
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img.astype(np.uint8), cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    e = MainWindow()
    sys.exit(app.exec_())