import sys
import serial
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
time.sleep(1) # Lets i2c connection start
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

ads = ADS.ADS1115(i2c) # Create ADS object - I2C Enabled on board pi
 
# Motor Variables
DIR = 5
STEP = 6
CW = 1
CCW = 0
spd = .001
ClampSteps = 10550 #Number of motor steps to clamp fully

# Motor Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# Limit Switch
ButtonPin = 12
GPIO.setup(ButtonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Create Serial Connection w/ Syringe Pump
try:
    ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate = 9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
    )
except:
    pass

# Global Variables
global data
data = {}

global go
go = False

global step
global rate1cmd
global rate2cmd
global rate3cmd

class Ui_MainWindow(QtWidgets.QMainWindow):
 
    def __init__(self):
 
        super().__init__()
        self.timer_thread = TimerThread()        
        self.timer_thread.update.connect(self.retranslateUi)
        self.pump_thread = PumpThread()
 
 
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
 
        # -------------------Motor Buttons -----------------------------
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
        self.motorButton = QtWidgets.QPushButton("Motor Close", self.centralwidget)
        self.motorButton.setGeometry(QtCore.QRect(160, 65, 150, 50))
        self.motorButton.setObjectName("motorButton")
        self.motorButton.setCheckable(False)
 
        self.motorButton.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.motorButton.clicked.connect(self.motorCW)  # Connect button to motor run function
 
        # Motor Run CCW
        self.motorButton2 = QtWidgets.QPushButton("Motor Open", self.centralwidget)
        self.motorButton2.setGeometry(QtCore.QRect(325, 65, 150, 50))
        self.motorButton2.setObjectName("motorButton2")
        self.motorButton2.setCheckable(False)
        self.motorButton2.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.motorButton2.clicked.connect(self.motorCCW)  # Connect button to motor run function

        # Motor Home Button
        self.motorHome = QtWidgets.QPushButton("Motor Home", self.centralwidget)
        self.motorHome.setGeometry(QtCore.QRect(325, 135, 150, 50))
        self.motorHome.setObjectName("motorHome")
        self.motorHome.setCheckable(False)
 
        self.motorHome.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.motorHome.clicked.connect(self.motorGoHome)  # Connect button to motorGoHome function
        
        # Motor Clamp Button
        self.motorClamp = QtWidgets.QPushButton("Motor Clamp", self.centralwidget)
        self.motorClamp.setGeometry(QtCore.QRect(160, 135, 150, 50))
        self.motorClamp.setObjectName("motorClamp")
        self.motorClamp.setCheckable(False)
 
        self.motorClamp.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.motorClamp.clicked.connect(self.motorGoClose)  # Connect button to motorGoHome function

    # ---------------Misc Buttons --------------------    
        # Save Data PushButton
        self.dataButton = QtWidgets.QPushButton("Save Data", self.centralwidget)
        self.dataButton.setGeometry(QtCore.QRect(525, 65, 150, 50))
        self.dataButton.setObjectName("dataButton")
        self.dataButton.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.dataButton.clicked.connect(self.dataSave)  # Connect button to dataSave function

        # Run Start Button
        self.runStart = QtWidgets.QPushButton("Run Start", self.centralwidget)
        self.runStart.setGeometry(QtCore.QRect(525, 135, 150, 50))
        self.runStart.setObjectName("runStart")
        self.runStart.setCheckable(False)
        self.runStart.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.runStart.clicked.connect(self.startRun)

    # ---------------Syringe Pump Buttons --------------------
        # Pump Label
        self.PumpLabel = QtWidgets.QLabel("Syringe Pump Settings",self.centralwidget)
        self.PumpLabel.setGeometry(QtCore.QRect(875, 5, 220, 30))
        self.PumpLabel.setObjectName("UnitsLabel")
        font = self.font()  # New line for font change
        font.setPointSize(14) # New line for font change
        self.PumpLabel.setFont(QtGui.QFont(font))

        # Rate Units Dropdown
        self.flowUnits = QtWidgets.QComboBox(self.centralwidget)
        self.flowUnits.setGeometry(QtCore.QRect(1050, 200, 100,50))
        self.flowUnits.setObjectName("flowUnits")
        self.flowUnits.addItems(['mL/min', 'mL/hr', 'uL/min', 'uL/hr'])
        #self.flowUnits.activated.connect(self.pumpSetup)

        # Step Duration Box
        self.stepTimeLabel = QtWidgets.QLabel(self.centralwidget)
        self.stepTimeLabel.setGeometry(QtCore.QRect(900, 50, 220, 20))
        self.stepTimeLabel.setObjectName("stepTimeLabel")
        self.stepTimeLabel.setText("Step Time (sec):")
 
        self.stepTime = QtWidgets.QSpinBox(self.centralwidget)
        self.stepTime.setObjectName("stepTime")
        self.stepTime.setRange(0,2000)
        self.stepTime.setSingleStep(1)
        self.stepTime.setValue(10)
        self.stepTime.setGeometry(QtCore.QRect(910, 70, 75, 50))
        
        # Rate Boxes
        self.Rate1Label = QtWidgets.QLabel("Rate 1:",self.centralwidget)
        self.Rate1Label.setGeometry(QtCore.QRect(830, 140, 100, 50))
        self.Rate1Label.setObjectName("Rate1Label")
        self.Rate1 = QtWidgets.QLineEdit(self.centralwidget)
        self.Rate1.setObjectName("Rate1")
        self.Rate1.setGeometry(QtCore.QRect(900,150,100,40))
        self.Rate1.setText('0')

        self.Rate2Label = QtWidgets.QLabel("Rate 2:",self.centralwidget)
        self.Rate2Label.setGeometry(QtCore.QRect(830, 190, 100, 50))
        self.Rate2Label.setObjectName("Rate2Label")
        self.Rate2 = QtWidgets.QLineEdit(self.centralwidget)
        self.Rate2.setObjectName("Rate2")
        self.Rate2.setGeometry(QtCore.QRect(900,200,100,40))
        self.Rate2.setText('0')

        self.Rate3Label = QtWidgets.QLabel("Rate 3:",self.centralwidget)
        self.Rate3Label.setGeometry(QtCore.QRect(830, 240, 100, 50))
        self.Rate3Label.setObjectName("Rate3Label")
        self.Rate3 = QtWidgets.QLineEdit(self.centralwidget)
        self.Rate3.setObjectName("Rate3")
        self.Rate3.setGeometry(QtCore.QRect(900,250,100,40))
        self.Rate3.setText('0')

        # Confirm Box
        self.pumpConfirm = QtWidgets.QPushButton("Confirm Pump Setup", self.centralwidget)
        self.pumpConfirm.setGeometry(QtCore.QRect(870, 320, 160, 50))
        self.pumpConfirm.setObjectName("pumpConfirm")
        self.pumpConfirm.clicked.connect(self.show)
        QtCore.QMetaObject.connectSlotsByName(self)
        self.pumpConfirm.clicked.connect(self.pumpSetup)

    # -------------- -Graph Code -----------------------------
 
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
            if GPIO.input(ButtonPin)==GPIO.HIGH:
                break
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

    def motorGoHome(self):
        GPIO.output(DIR,CCW)
        # Run for X steps. This will change based on how you set you controller
        while GPIO.input(ButtonPin)==GPIO.LOW:
            # Set one coil winding to high
            GPIO.output(STEP,GPIO.HIGH)
            # Allow it to get there.
            sleep(spd) # Dictates how fast stepper motor will run
            # Set coil winding to low
            GPIO.output(STEP,GPIO.LOW)
            sleep(spd) # Dictates how fast stepper motor will run
        GPIO.cleanup

    def motorGoClose(self):
        GPIO.output(DIR,CW)
        # Run for 200 steps. This will change based on how you set you controller
        for x in range(ClampSteps-1):
            # Set one coil winding to high
            GPIO.output(STEP,GPIO.HIGH)
            # Allow it to get there.
            sleep(spd) # Dictates how fast stepper motor will run
            # Set coil winding to low
            GPIO.output(STEP,GPIO.LOW)
            sleep(spd) # Dictates how fast stepper motor will run
        GPIO.cleanup
    
    def startRun(self):
        global go
        go = True

        self.pump_thread.start()


    def pumpSetup(self):
        global step
        global rate1cmd
        global rate2cmd
        global rate3cmd

        units = self.flowUnits.currentText() # Units of flow rate #str
        step = self.stepTime.value() #int
        rate1val = self.Rate1.text() #str
        rate2val = self.Rate2.text() #str
        rate3val = self.Rate3.text() #str
        print([units,step,rate1val,rate2val,rate3val])
        
        unitDict = {"mL/min":"MM", "mL/hr":"MH", "uL/min":"UM", "uL/hr":"UH"}
        rate1cmd = '*RAT' + rate1val + unitDict[units] + '\x0D'
        rate2cmd = '*RAT' + rate2val + unitDict[units] + '\x0D'
        rate3cmd = '*RAT' + rate3val + unitDict[units] + '\x0D'


    def retranslateUi(self):
 
        global data
        global go

        self.PressureLabel1.setText(self._translate("MainWindow", f"Pressure Sensor 1: {data['p_sensor_1']}"))
        self.PressureLabel2.setText(self._translate("MainWindow", f"Pressure Sensor 2: {data['p_sensor_2']}"))
        self.PressureDiff.setText(self._translate("MainWindow", f"Pressure Drop = {data['p_diff']}"))
        
        if go:
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
            Voltage1 = chan1.voltage
            Voltage2 = chan2.voltage
            data["p_sensor_1"] = (((Voltage1-0.2)*60)/1.6)
            data["p_sensor_2"] =  0
            data["p_diff"] = data["p_sensor_1"] - data["p_sensor_2"]
            data["flow_rate"] = FlowRateX
            self.update.emit()


class PumpThread(QThread): # Doesn't do anything yet
    update = pyqtSignal()
 
    def __init__(self):
        QThread.__init__(self)
        global step
        global rate1cmd
        global rate2cmd
        global rate3cmd

    def run(self): # Executes to run the syringe pump
        
        cmd_run = b'*RUN\x0D'
        cmd_stop = b'*STP\x0D'
        run_time = step

        # Rate 1
        ser.write(rate1cmd.encode()) # Send Rate 1
        sleep(1)
        ser.write(cmd_run) # Start running
        sleep(run_time) # Don't send anything for duration
        ser.write(cmd_stop)# Stop after the unit time

        # Rate 2
        ser.write(rate2cmd.encode()) # Send Rate 2
        sleep(1)
        ser.write(cmd_run) # Start running
        sleep(run_time) # Don't send anything for duration
        ser.write(cmd_stop)# Stop after the unit time

        # Rate 3
        ser.write(rate3cmd.encode()) # Send Rate 3
        sleep(1)
        ser.write(cmd_run) # Start running
        sleep(run_time) # Don't send anything for duration
        ser.write(cmd_stop)# Stop after the unit time



app = QApplication(sys.argv)


window = Ui_MainWindow()
window.setupUi()
window.show()
sys.exit(app.exec())
 
# threading.Thread(target=update, daemon = True).start() vsCodetestttttt bro
