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

class EmerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(640, 480)
        self.setWindowTitle("위험 상황")
        self.setStyleSheet("background-color: white")
        self.installEventFilter(self)
        
        self.bat_percent = 100
        self.cpu_temp = 10
        
        self.bat_stat = QLabel('배터리상태: {}%'.format(str(self.bat_percent)), self)
        self.bat_stat.setFont(QFont('NanumGothic', 12))
        self.bat_stat.setGeometry(135, 10, 200, 30)
        self.cpu_stat = QLabel('CPU: {}℃'.format(str(self.cpu_temp)), self)
        self.cpu_stat.setFont(QFont('NanumGothic', 12))
        self.cpu_stat.setGeometry(400, 10, 200, 30)

        self.rgb_img_label = QLabel(self)
        self.rgb_img_label.setGeometry(95, 40, 640 * 0.7, 480 * 0.7)

        self.depth_img_label = QLabel(self)
        self.depth_img_label.setGeometry(95, 40, 640 * 0.7, 480 * 0.7)

        self.person_detect_msg = QLabel("요구조자를 탐색중입니다.", self)
        self.person_detect_msg.setFont(QFont('NanumGothic', 12))
        self.person_detect_msg.setGeometry(230, 390, 180, 20)

        self.detection_rgb_img_btn = QPushButton('감지 이미지 보기', self)
        self.detection_rgb_img_btn.setGeometry(170, 430, 120, 30)
        self.detection_rgb_img_btn.setStyleSheet("background-color: white")

        self.detection_depth_img_btn = QPushButton('Depth 이미지 보기', self)
        self.detection_depth_img_btn.setGeometry(350, 430, 120, 30)
        self.detection_depth_img_btn.setStyleSheet("background-color: white")

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
        p = convert_to_Qt_format.scaled(640 * 0.7, 480 * 0.7, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    e = EmerWindow()
    sys.exit(app.exec_())
