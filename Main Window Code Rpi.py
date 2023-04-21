import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import numpy as np
import random
import time
from time import sleep
import threading
import RPi.GPIO as GPIO
# ADC Libraries
import board
import busio
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1) # Lets i2c connection form?
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

ads = ADS.ADS1115(i2c) # Create ADS object - I2C Enabled on board pi
 
# Motor Variables
DIR = 5
STEP = 6
CW = 1
CCW = 0
spd = .001
# Motor Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# Global Variables


class Ui_MainWindow(QtWidgets.QMainWindow):
 
    def __init__(self):
 
        super().__init__()
        self.timer_thread = TimerThread()        
        self.timer_thread.update.connect(self.retranslateUi)
 
 
    def setupUi(self):
 
        self.setObjectName("MainWindow")
        self.resize(700, 500)  # Initial size of popup window
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
 
        self.PressureLabel1 = QtWidgets.QLabel(self.centralwidget)
        self.PressureLabel1.setGeometry(QtCore.QRect(1, 15, 220, 20))
        self.PressureLabel1.setObjectName("PressureLabel1")
 
 
        self.PressureLabel2 = QtWidgets.QLabel(self.centralwidget)
        self.PressureLabel2.setObjectName("PressureLabel2")
        self.PressureLabel2.setGeometry(QtCore.QRect(250, 15, 220, 20))
 
 
        self.PressureDiff = QtWidgets.QLabel(self.centralwidget)
        self.PressureDiff.setObjectName("PressureDiff")
        self.PressureDiff.setGeometry(QtCore.QRect(490, 15, 220, 20))
 
        self.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 411, 18))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)
 
        # -------------------- Motor Buttons -----------------------------
        # Step Size Box
        self.spinLabel = QtWidgets.QLabel(self.centralwidget)
        self.spinLabel.setGeometry(QtCore.QRect(1, 65, 220, 20))
        self.spinLabel.setObjectName("spinLabel")
        self.spinLabel.setText("Motor Step Size:")
 
        self.spin = QtWidgets.QSpinBox(self.centralwidget)
        self.spin.setObjectName("spin")
        self.spin.setRange(0,2000)
        self.spin.setSingleStep(100)
        self.spin.setValue(100)
        self.spin.setGeometry(QtCore.QRect(20, 95, 75, 50))
 
        # Motor Run CW
        self.motorButton = QtWidgets.QPushButton("Motor CW", self.centralwidget)
        self.motorButton.setGeometry(QtCore.QRect(160, 65, 150, 50))
        self.motorButton.setObjectName("motorButton")
        self.motorButton.setCheckable(False)
 
        self.motorButton.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.motorButton.clicked.connect(self.motorCW)  # Connect button to motor run function
 
        # Motor Run CCW
        self.motorButton2 = QtWidgets.QPushButton("Motor CCW", self.centralwidget)
        self.motorButton2.setGeometry(QtCore.QRect(325, 65, 150, 50))
        self.motorButton2.setObjectName("motorButton2")
        self.motorButton2.setCheckable(False)
        self.motorButton2.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.motorButton2.clicked.connect(self.motorCCW)  # Connect button to motor run function

        # Save Data PushButton
        self.dataButton = QtWidgets.QPushButton("Save Data", self.centralwidget)
        self.dataButton.setGeometry(QtCore.QRect(525, 65, 150, 50))
        self.dataButton.setObjectName("dataButton")
        self.dataButton.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.dataButton.clicked.connect(self.dataSave)  # Connect button to dataSave function
 
 
 
    # ---------------Graphing Stuff Testing ------------------
 
        self.graphWidget = pg.PlotWidget(self)
 
        #                                         x  y xSize ySize
        self.graphWidget.setGeometry(QtCore.QRect(1,220,800,260))
 
        # plot data and adjust graphics
        self.graphWidget.setBackground([191, 189, 182])
        self.graphWidget.setTitle("Pressure Drop vs Flow Rate", color="b", size="12pt")
        styles = {'color':'r', 'font-size':'10pt'}
        self.graphWidget.setLabel('left', 'Pressure Drop (psi)', **styles)
        self.graphWidget.setLabel('bottom', 'Time (s)', **styles)
        self.pen = pg.mkPen(color=(255, 0, 0), width = 6)
        # self.graphWidget.resize(200,200)
 
        self._translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(self._translate("MainWindow", "Flow Device"))
        font = self.font()  # New line for font change
        font.setPointSize(12) # New line for font change
        self.PressureLabel1.setFont(QtGui.QFont(font))
        self.PressureLabel2.setFont(QtGui.QFont(font))
        self.PressureDiff.setFont(QtGui.QFont(font))
 
        self.PDrop = []
        self.flowRate = []
 
        self.dataPlot = self.graphWidget.plot(self.flowRate, self.PDrop, pen=self.pen)
 
        self.timer_thread.start()
 
    def motorCW(self):
 
        GPIO.output(DIR,CW)
        # Run for 200 steps. This will change based on how you set you controller
        for x in range(self.spin.value()):
            # Set one coil winding to high
            GPIO.output(STEP,GPIO.HIGH)
            # Allow it to get there.
            sleep(spd) # Dictates how fast stepper motor will run
            # Set coil winding to low
            GPIO.output(STEP,GPIO.LOW)
            sleep(spd) # Dictates how fast stepper motor will run
        GPIO.cleanup
 
    def motorCCW(self):
 
        GPIO.output(DIR,CCW)
        # Run for 200 steps. This will change based on how you set you controller
        for x in range(self.spin.value()):
            # Set one coil winding to high
            GPIO.output(STEP,GPIO.HIGH)
            # Allow it to get there.
            sleep(spd) # Dictates how fast stepper motor will run
            # Set coil winding to low
            GPIO.output(STEP,GPIO.LOW)
            sleep(spd) # Dictates how fast stepper motor will run
        GPIO.cleanup
 
    def dataSave(self):
        txtsave = np.column_stack([self.flowRate,self.PDrop])
        np.savetxt('RunData.txt', txtsave, fmt = ['%5.2f','%5.2f'])


    def retranslateUi(self):
 
        global data
 
        self.PressureLabel1.setText(self._translate("MainWindow", f"Pressure Sensor 1: {data['p_sensor_1']}"))
        self.PressureLabel2.setText(self._translate("MainWindow", f"Pressure Sensor 2: {data['p_sensor_2']}"))
        self.PressureDiff.setText(self._translate("MainWindow", f"Pressure Drop = {data['p_diff']}"))
 
        self.flowRate.append(data["flow_rate"])
        self.PDrop.append(data["p_diff"])
 
        self.dataPlot.setData(self.flowRate, self.PDrop)
 
 
 
class TimerThread(QThread):
    update = pyqtSignal()
 
    def __init__(self):
        QThread.__init__(self)
 
    def run(self):
        global data
        FlowRateX = 1
 
        while True:
            time.sleep(1)
#           FlowRateX = 1
            FlowRateX +=1
            chan1 = AnalogIn(ads, ADS.P0) # Pin 0 - A0
            chan2 = AnalogIn(ads, ADS.P1) # Pin 1 - A1
            Voltage1 = chan1.voltage * 1.8
            Voltage2 = chan2.voltage * 1.8
            data["p_sensor_1"] = (((Voltage1-0.2)*30)/1.6)
            data["p_sensor_2"] =  0
            data["p_diff"] = data["p_sensor_1"] - data["p_sensor_2"]
            data["flow_rate"] = FlowRateX
            self.update.emit()
 
app = QApplication(sys.argv)
global data
data = {}
 
 
 
window = Ui_MainWindow()
window.setupUi()
window.show()
sys.exit(app.exec())
 
# threading.Thread(target=update, daemon = True).start() vsCodetestttttt bro
