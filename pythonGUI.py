from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import example_ui as ui
import serial

class ScoreThread(QThread):

    trigger = pyqtSignal(str)

    def __init__(self):
        QThread.__init__(self)

    def run(self):
        ser = serial.Serial("COM5", 115200)
        sline = ser.readline()
        ser.close()
        self.trigger.emit(sline.decode('utf-8').strip('\x00'))

class Main(QMainWindow, ui.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pushButton.clicked.connect(self.start)
        self.pushButton_2.clicked.connect(self.retry)

    def start(self):
        #print("start")
        ser = serial.Serial("COM5", 115200)
        ser.write(b'1')
        ser.close()
        self.score = ScoreThread()
        self.score.trigger.connect(self.handleScore)
        self.score.start()
        self.pushButton.setEnabled(False)
        self.pushButton_2.setEnabled(False)
    
    def retry(self):
        #print("retry")
        self.label_3.setText("0")
        ser = serial.Serial("COM5", 115200)
        ser.write(b'1')
        ser.close()
        self.score = ScoreThread()
        self.score.trigger.connect(self.handleScore)
        self.score.start()
        self.pushButton.setEnabled(False)
        self.pushButton_2.setEnabled(False)
    
    def about_msg(self, message):
        reply = QMessageBox.about(self, "標題", "<font color='white'><p>" + message + "</p> ")
    
    @pyqtSlot(str) 
    def handleScore(self, message): 
        self.label_3.setText(message)
        self.pushButton.setEnabled(True)
        self.pushButton_2.setEnabled(True)



if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Main()
    window.show()
    sys.exit(app.exec_())