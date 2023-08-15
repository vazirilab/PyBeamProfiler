from PyQt5.QtWidgets import *
from PyQt5 import uic
# from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import QtGui, QtCore
import skimage
import numpy as np
import sys
import glob
import os
import PIL
# from PIL import ImageFilter, Image
import math
import time
import scipy as sp
import PySpin
from View_page import Ui_MainWindow
from Offline_viewer import Ui_Ofline_Viewer
from Arduino_Control import Ui_Arduino_Control
import pyqtgraph as pg
import pyfirmata
import serial
import serial.tools.list_ports

# A thread for the camera. It emits frames and the number of the current frame
class Thread_FLIR(QtCore.QThread):
    changePixmap_FLIR = QtCore.pyqtSignal(np.ndarray, int, int) # a signal to renew each frame
    # those values are initial numbers that will be redefined in the GUI
    NUM_IMG = 100000
    serial = '20270803'  # seriel of the flir cam
    ppmm = 0.0069  # pixels per mm to convert position to mm
    ppmm = 1 / ppmm
    frame_rate = 50.0
    ExpTime_us = 8000
    Stop_Loop = False
    file_names = None
    Auto_ExpTimeCond = False
    def run(self):

        self.system = PySpin.System.GetInstance()  # Retrieve singleton reference to system object
        self.version = self.system.GetLibraryVersion()  # Get current library version
        self.cam_list = self.system.GetCameras()  # Retrieve list of cameras from the system
        self.cam = self.cam_list.GetBySerial(self.serial)
        sNodemap = self.cam.GetTLStreamNodeMap() #change the streaming mode to acquire new frames only
        node_bufferhandling_mode = PySpin.CEnumerationPtr(sNodemap.GetNode('StreamBufferHandlingMode'))
        node_newestonly = node_bufferhandling_mode.GetEntryByName('NewestOnly')
        node_newestonly_mode = node_newestonly.GetValue()
        node_bufferhandling_mode.SetIntValue(node_newestonly_mode)
        self.nodemap_tldevice = self.cam.GetTLDeviceNodeMap()
        self.cam.Init()
        self.nodemap = self.cam.GetNodeMap()
        self.node_acquisition_mode = PySpin.CEnumerationPtr(self.nodemap.GetNode('AcquisitionMode'))
        self.node_acquisition_mode_continuous = self.node_acquisition_mode.GetEntryByName('Continuous')
        self.acquisition_mode_continuous = self.node_acquisition_mode_continuous.GetValue()
        self.node_acquisition_mode.SetIntValue(self.acquisition_mode_continuous)
        self.processor = PySpin.ImageProcessor()
        self.cam.BeginAcquisition()
        self.cam.ExposureTime.SetValue(self.ExpTime_us)
        self.cam.AcquisitionFrameRateEnable.SetValue(True)
        self.cam.AcquisitionFrameRate.SetValue(self.frame_rate)
        i = 0
        while i <= (self.NUM_IMG - 1):
            if self.Stop_Loop:
                break

            image_result = self.cam.GetNextImage(10000)  # capturing a frame
            FrameNP = image_result.GetNDArray()  # get the result as a numpy array
            TimeStamp = image_result.GetTimeStamp()
            if np.max(FrameNP) > 20:  # prevent gui from crashing when laser off

                # setting Auto exposure time mode
                if self.Auto_ExpTimeCond and np.max(FrameNP) > 240:

                    self.ExpTime_us = self.ExpTime_us - 100
                    self.cam.ExposureTime.SetValue(self.ExpTime_us)
                    image_result = self.cam.GetNextImage(10000)  # capturing a frame
                    FrameNP = image_result.GetNDArray()  # get the result as a numpy array
                    time.sleep(0.01)


                elif np.max(FrameNP) < 120 and self.Auto_ExpTimeCond:

                    self.ExpTime_us = self.ExpTime_us + 50
                    self.cam.ExposureTime.SetValue(self.ExpTime_us)
                    image_result = self.cam.GetNextImage(10000)  # capturing a frame
                    FrameNP = image_result.GetNDArray()  # get the result as a numpy array
                    time.sleep(0.01)

                self.changePixmap_FLIR.emit(FrameNP, i, TimeStamp)  # send the frame and the current frame number to the GUI
                i = i + 1
                image_result.Release()
            else:
                print(f"No laser detected, Please turn on the laser. \n")
                while True:
                    image_result = self.cam.GetNextImage(10000)  # capturing a frame
                    FrameNP = image_result.GetNDArray()  # get the result as a numpy array
                    time.sleep(0.01)
                    image_result.Release()
                    if np.max(FrameNP) > 20:
                        if i == 0:
                            print(f"Laser detected. \n ")
                            break
                        else:
                            print(f"Laser detected. \n ")
                            i = i - 1
                            break

                    if self.Stop_Loop:
                        break

            time.sleep(0.01)

        self.cam.EndAcquisition()
        self.cam.DeInit()

#class Thread_Web_Cam(QtCore.QThread):
#    changePixmap_Web_Cam = QtCore.pyqtSignal(np.ndarray, int) # a signal to renew each frame
#    # those values are initial numbers that will be redefined in the GUI
#    NUM_IMG = 100000
#    serial = '20270803'  # seriel of the flir cam
#    ppmm = 0.0069  # pixels per mm to convert position to mm
#    ppmm = 1 / ppmm
#    frame_rate = 50.0
#    ExpTime_us = 8000
#    Stop_Loop = False
#    file_names = None
#
#    def run(self):
#
#        self.vid = cv2.VideoCapture(self.serial)
#        self.cam.Init()
#        self.processor = PySpin.ImageProcessor()
#        self.cam.BeginAcquisition()
#        self.cam.ExposureTime.SetValue(self.ExpTime_us)
#        for i in range(self.NUM_IMG):
#            if self.Stop_Loop:
#                break
#
                # Capture the video frame
                # by frame
 #           self.ret, self.frame = self.vid.read()
 #           FrameNP = image_result.GetNDArray()  # get the result as a numpy array

#            time.sleep(1 / self.frame_rate)
#            self.changePixmap_FLIR.emit(FrameNP, i) # send the frame and the current frame number to the GUI
#        self.cam.EndAcquisition()
#        self.cam.DeInit()


class Thread_Data(QtCore.QThread):
    changePixmap_Data = QtCore.pyqtSignal(np.ndarray, int)# a signal to renew each frame
    # those values are initial numbers that will be redefined in the GUI
    NUM_IMG = 100000
    serial = '20270803'  # seriel of the flir cam
    ppmm = 0.0069  # pixels per mm to convert position to mm
    ppmm = 1 / ppmm
    frame_rate = 50.0
    ExpTime_us = 8000
    Stop_Loop = False
    file_names = None
    Auto_ExpTimeCond = False
    def run(self):
        file_Nr = len(self.file_names)
        for i in range(file_Nr):
            FrameNP = skimage.io.imread(self.file_names[i])  # get the result as a numpy array
            if self.Stop_Loop:
                break
            time.sleep(1 / self.frame_rate)
            self.changePixmap_Data.emit(FrameNP, i)  # send the frame and the current frame number to the GUI


class PyBeamProfilerGUI(QMainWindow):
    def __init__(self):
        super(PyBeamProfilerGUI, self).__init__()
        uic.loadUi("Main_page.ui", self)
        self.show()
        self.print_cam_serial_info.clicked.connect(self.cam_serial_info)
        self.print_nr_of_frames_info.clicked.connect(self.nr_of_frames_info)
        self.print_pixel_size_info.clicked.connect(self.pixel_size_info)
        self.Open_Viewer.clicked.connect(self.OpenViewer)
        self.start_acquisition.clicked.connect(self.StartAcquisition)
        self.ExpTimeInfo.clicked.connect(self.exp_time_info)
        self.FrameRateInfo.clicked.connect(self.frame_rate_info)
        self.StreamType_info.clicked.connect(self.stream_type_info)
        self.stop_acquisition.clicked.connect(self.StopAcquisition)
        self.actionOpen.triggered.connect(self.OpenFile)
        self.actionAnalysis_Methode.triggered.connect(self.PrintAnalysisInfo)
        self.actionAbout.triggered.connect(self.PrintAboutInfo)
        self.actionPosition_Control.triggered.connect(self.Open_Ard_Window)
        self.cam_serial.setText('20270803')
        self.nr_of_frames.setText('1000')
        self.pixel_size.setText('0.0069')
        self.ExpTime_text.setText('3000')
        self.FrameRate_text.setText('30')
        self.FramesPerFile_text.setText('1000')
        self.cam_serial.textChanged.connect(self.CheckForInt_cam_serial)
        self.nr_of_frames.textChanged.connect(self.CheckForInt_nr_of_frames)
        self.pixel_size.textChanged.connect(self.CheckForInt_pixel_size)
        self.ExpTime_text.textChanged.connect(self.CheckForInt_ExpTime_text)
        self.FrameRate_text.textChanged.connect(self.CheckForInt_FrameRate_text)
        self.FramesPerFile_text.textChanged.connect(self.CheckForInt_FramesPerFile_text)
        self.SavingOption.stateChanged.connect(self.Saving_Option)
        self.AutoExpTime.stateChanged.connect(self.Auto_ExpTime)
        self.PositionOnly.stateChanged.connect(self.Position_Only)
        self.SavedFileNumber = 0
        self.RemainingFrames = None
        self.CurrentFrame = 0
        self.StartingTime = 0

        # Stream View Window
        self.ui = Ui_MainWindow()
        self.window1 = QMainWindow()
        self.ui.setupUi(self.window1)
        self.ui.X_Pos_Plot.clicked.connect(self.PlotXchange)
        self.ui.Y_Pos_Plot.clicked.connect(self.PlotYchange)
        self.ui.FWHM_Plot.clicked.connect(self.PlotFWHMchange)
        self.ui.Std_LA_Plot.clicked.connect(self.PlotStdchange)


        # Offline View Window
        self.ui_offline = Ui_Ofline_Viewer()
        self.window2 = QMainWindow()
        self.ui_offline.setupUi(self.window2)


        # Arduino control
        self.ui_Arduino = Ui_Arduino_Control()
        self.window3 = QMainWindow()
        self.ui_Arduino.setupUi(self.window3)
        self.ui_Arduino.calibrate_motors.clicked.connect(self.Caliberate_Motors)
        self.ui_Arduino.start_acquisition_ard.clicked.connect(self.StartAcquisition_ard)

        # Variables
        self.X_Max_Pos = None  # array to follow the X-position of the beam center
        self.Y_Max_Pos = None  # array to follow the Y-position of the beam center
        self.pos2 = None
        self.std2 = None
        self.FWHM = None
        self.LongAxisPlot = None
        self.FrameTime = None
        self.allfiles = None
        self.Nr_allfiles = None
        self.X_Max_Pos_offline = None  # array to follow the X-position of the beam center
        self.Y_Max_Pos_offline = None  # array to follow the Y-position of the beam center
        self.Circ_offline = None
        self.Y_Plot_offline = None
        self.X_Plot_offline = None
        self.DataFileName = None
        self.StdPlot = None
        self.window_plot_Std = None
        self.Y_Plot = None
        self.window_plot_y = None
        self.X_Plot = None
        self.window_plot_x = None
        self.window_plot_y_offline = None
        self.window_plot_x_offline = None
        self.thorlabs_AnglePerRev = 8e-3  # rad
        self.th = Thread_FLIR(self)
        self.caliberate_movment_values = False
        self.ard_board = 0

    def Follow_Position(self, FrameNP, i, TimeStamp):
        self.ui_Arduino.cam_view.setPixmap(QtGui.QPixmap(QtGui.QPixmap.fromImage(self.gray2qimage(FrameNP))))
        FrameNP = (FrameNP - np.min(FrameNP)) / (np.max(FrameNP) - np.min(FrameNP))  # normalize the matrix
        # the elliptic shape of the beam is extracted on the form of points on a certain intensity of the gaussian beam
        upper_limit = 0.505
        lower_limit = 0.495
        cond = True  # makes sure the elipse is detected
        while cond:
            yx_coords = np.column_stack(np.where((FrameNP >= lower_limit) & (FrameNP <= upper_limit)))
            if np.max(np.shape(yx_coords)) > 2:
                upper_limit = 0.505
                lower_limit = 0.495
                cond = False
            else:
                upper_limit = upper_limit + 0.005
                lower_limit = lower_limit - 0.005
            if (upper_limit > 1) or (lower_limit < 0):
                print('Make sure camera is open')
                cond = False
        self.CurrentFrame = i - self.SavedFileNumber * int(self.FramesPerFile_text.text())
        Y_Center = ((np.amax(yx_coords[:, 0]) + np.amin(yx_coords[:, 0])) / 2)
        X_Center = ((np.amax(yx_coords[:, 1]) + np.amin(yx_coords[:, 1])) / 2)
        self.X_Max_Pos[self.CurrentFrame] = X_Center * float(self.pixel_size.text())
        self.Y_Max_Pos[self.CurrentFrame] = Y_Center * float(self.pixel_size.text())





        self.FrameTime[self.CurrentFrame] = TimeStamp
        self.ui.x_position_text.setText(str(self.X_Max_Pos[self.CurrentFrame]))
        self.ui.y_position_text.setText(str(self.Y_Max_Pos[self.CurrentFrame]))
        if self.SavingOption.isChecked() and ((self.CurrentFrame == int(self.nr_of_frames.text())) or
                                              (self.CurrentFrame == self.FileStreamNr - 1) or
                                              (self.CurrentFrame == int(self.FramesPerFile_text.text()) - 1) or
                                              self.RemainingFrames == int(self.nr_of_frames.text()) -
                                              int(self.FramesPerFile_text.text()) * self.SavedFileNumber):

            if (self.CurrentFrame == int(self.FramesPerFile_text.text()) - 1) and \
                    (int(self.nr_of_frames.text()) > int(self.FramesPerFile_text.text())):
                np.savetxt(self.DataFileName[0: len(self.DataFileName) - 4] + str(self.SavedFileNumber) + '.csv',
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos],
                           delimiter=",")

                self.SavedFileNumber = self.SavedFileNumber + 1
                self.RemainingFrames = self.RemainingFrames - int(self.FramesPerFile_text.text())
                self.X_Max_Pos = np.zeros(
                    int(self.FramesPerFile_text.text()))  # array to follow the X-position of the beam center
                self.Y_Max_Pos = np.zeros(
                    int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center

            elif (self.RemainingFrames > 0) and (self.RemainingFrames < int(self.FramesPerFile_text.text())) and \
                    self.RemainingFrames == int(self.nr_of_frames.text()) - \
                    int(self.FramesPerFile_text.text()) * self.SavedFileNumber:
                self.stop_acquisition.setEnabled(False)
                np.savetxt(self.DataFileName[0: len(self.DataFileName) - 4] + str(self.SavedFileNumber) + '.csv',
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos],
                           delimiter=",")


            elif self.CurrentFrame == int(self.nr_of_frames.text()) - 1:
                self.stop_acquisition.setEnabled(False)
                np.savetxt(self.DataFileName,
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos],
                           delimiter=",")

    def StartAcquisition_ard(self):
        self.stop_acquisition.setEnabled(True)
        self.CurrentFrame = 0
        self.SavedFileNumber = 0
        self.printed_info.setText('   Acquisition starting... Please press Open Viewer to see the camera video stream.')
        self.StartingTime = time.time()
        self.th.terminate()
        self.th = Thread_FLIR(self)  # calling the thread of the camera
        self.th.changePixmap_FLIR.connect(self.Follow_Position)

        self.th.NUM_IMG = int(self.nr_of_frames.text())
        self.th.serial = (self.cam_serial.text())
        self.th.ExpTime_us = float(self.ExpTime_text.text())
        self.th.frame_rate = float(self.FrameRate_text.text())
        self.th.Auto_ExpTimeCond = self.AutoExpTime.isChecked()
        self.FileStreamNr = int(self.nr_of_frames.text())
        if int(self.nr_of_frames.text()) > int(self.FramesPerFile_text.text()) and self.SavingOption.isChecked():

            self.X_Max_Pos = np.zeros(
                 int(self.FramesPerFile_text.text()))  # array to follow the X-position of the beam center
            self.Y_Max_Pos = np.zeros(
                 int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center
            self.FrameTime = np.zeros(int(self.FramesPerFile_text.text()))

        else:
            self.X_Max_Pos = np.zeros(
                int(self.nr_of_frames.text()))  # array to follow the X-position of the beam center
            self.Y_Max_Pos = np.zeros(
                int(self.nr_of_frames.text()))  # array to follow the Y-position of the beam center
            self.FrameTime = np.zeros(int(self.nr_of_frames.text()))
        self.ui_Arduino.calibrate_motors.setEnabled(True)
        self.th.start()

    def Caliberate_Motors(self):
        if len(self.ui_Arduino.ENA_Pin_Y.text()) < 1:
            self.printed_info.setText('add a valid ENA Pin')
        elif len(self.ui_Arduino.ENA_Pin_X.text()) < 1:
            self.printed_info.setText('add a valid ENA Pin')
        elif len(self.ui_Arduino.PUL_Pin_X.text()) < 1:
            self.printed_info.setText('add a valid PUL Pin')
        elif len(self.ui_Arduino.PUL_Pin_Y.text()) < 1:
            self.printed_info.setText('add a valid PUL Pin')
        elif len(self.ui_Arduino.DIR_Pin_X.text()) < 1:
            self.printed_info.setText('add a valid DIR Pin')
        elif len(self.ui_Arduino.DIR_Pin_Y.text()) < 1:
            self.printed_info.setText('add a valid DIR Pin')
        elif len(self.ui_Arduino.OPTO_Pin_Y.text()) < 1:
            self.printed_info.setText('add a valid OPTO Pin')
        elif len(self.ui_Arduino.OPTO_Pin_X.text()) < 1:
            self.printed_info.setText('add a valid OPTO Pin')
        elif len(self.ui_Arduino.ARD_Port.text()) < 1:
            self.printed_info.setText('add a valid Arduino port ')
        elif len(self.ui_Arduino.StepsPerRev.text()) < 1:
            self.printed_info.setText('add a valid Steps / rev. number')
        else:
            if self.ard_board == 0:
                self.ard_board = pyfirmata.Arduino(self.ui_Arduino.ARD_Port.text())
            self.ENAY = int(self.ui_Arduino.ENA_Pin_Y.text())
            self.ENAX = int(self.ui_Arduino.ENA_Pin_X.text())
            self.DIRY = int(self.ui_Arduino.DIR_Pin_Y.text())
            self.DIRX = int(self.ui_Arduino.DIR_Pin_X.text())
            self.PULX = int(self.ui_Arduino.PUL_Pin_X.text())
            self.PULY = int(self.ui_Arduino.PUL_Pin_Y.text())
            self.OPTOX = int(self.ui_Arduino.OPTO_Pin_X.text())
            self.OPTOY = int(self.ui_Arduino.OPTO_Pin_Y.text())

            if self.CurrentFrame < 5:
                self.meanX5_1 = np.mean(self.X_Max_Pos[self.CurrentFrame - 5: self.CurrentFrame])

                self.rotate_stepper(int(int(self.ui_Arduino.StepsPerRev.text()) / 10), 0, self.ard_board, self.ENAX, self.DIRX,
                                    self.PULX, self.OPTOX)
                self.meanX5_2 = np.mean(self.X_Max_Pos[self.CurrentFrame - 5: self.CurrentFrame])
                self.rotate_stepper(int(int(self.ui_Arduino.StepsPerRev.text()) / 10), 1, self.ard_board, self.ENAX, self.DIRX,
                                    self.PULX, self.OPTOX)
                self.meanY5_1 = np.mean(self.Y_Max_Pos[self.CurrentFrame - 5: self.CurrentFrame])
                self.rotate_stepper(int(int(self.ui_Arduino.StepsPerRev.text()) / 10), 0, self.ard_board, self.ENAY, self.DIRY,
                                    self.PULY, self.OPTOY)
                self.meanY5_2 = np.mean(self.Y_Max_Pos[self.CurrentFrame - 5: self.CurrentFrame])
                self.rotate_stepper(int(int(self.ui_Arduino.StepsPerRev.text()) / 10), 1, self.ard_board, self.ENAY, self.DIRY,
                                    self.PULY, self.OPTOY)
                self.ui_Arduino.up_button.setEnabled(True)
                self.ui_Arduino.down_button.setEnabled(True)
                self.ui_Arduino.left_button.setEnabled(True)
                self.ui_Arduino.right_button.setEnabled(True)

    def keyPressEvent(self, event):

        key = event.key()
        if not (self.ard_board == 0):
            if key == QtCore.Qt.Key_S:
                if event.isAutoRepeat():
                    if self.k == 1:
                        self.cond_motor_y = True
                        self.rotate_stepper_continuous_Y(0, self.ard_board, self.ENAY, self.DIRY, self.PULY, self.OPTOY)
                        self.k = 0
                else:
                    self.rotate_stepper( 1, 0, self.ard_board, self.ENAY, self.DIRY, self.PULY, self.OPTOY)
                    self.k = 1
            if key == QtCore.Qt.Key_W:
                if event.isAutoRepeat():
                    if self.k == 1:
                        self.cond_motor_y = True
                        self.rotate_stepper_continuous_Y(0, self.ard_board, self.ENAY, self.DIRY, self.PULY, self.OPTOY)
                        self.k = 0
                else:
                    self.rotate_stepper( 1, 1, self.ard_board, self.ENAY, self.DIRY, self.PULY, self.OPTOY)
                    self.k = 1
            if key == QtCore.Qt.Key_D:
                if event.isAutoRepeat():
                    if self.k == 1:
                        self.cond_motor_x = True
                        self.rotate_stepper_continuous_X(0, self.ard_board, self.ENAX, self.DIRX, self.PULX, self.OPTOX)
                        self.k = 0
                else:
                    self.rotate_stepper( 1, 0, self.ard_board, self.ENAX, self.DIRX, self.PULX, self.OPTOX)
                    self.k = 1
            if key == QtCore.Qt.Key_A:
                if event.isAutoRepeat():
                    if self.k == 1:
                        self.cond_motor_x = True
                        self.rotate_stepper_continuous_X(1, self.ard_board, self.ENAX, self.DIRX, self.PULX, self.OPTOX)
                        self.k = 0
                else:
                    self.rotate_stepper( 1, 0, self.ard_board, self.ENAX, self.DIRX, self.PULX, self.OPTOX)
                    self.k = 1

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_S and not event.isAutoRepeat():
            self.cond_motor_y = False
        if key == QtCore.Qt.Key_W and not event.isAutoRepeat():
            self.cond_motor_y = False
        if key == QtCore.Qt.Key_D and not event.isAutoRepeat():
            self.cond_motor_x = False
        if key == QtCore.Qt.Key_A and not event.isAutoRepeat():
            self.cond_motor_x = False

    def Open_Ard_Window(self):
        # Find the Arduino port(s).
        arduino_ports = self.find_arduino_ports()

        if len(arduino_ports) > 0:
            for port, description in arduino_ports:
                self.printed_info.setText(f"Arduino detected: {description} on port {port}\n")
                self.window3.show()
        else:
            self.printed_info.setText("No Arduinos detected.")
            self.window3.show()############

    def find_arduino_ports(self):
        arduino_ports = [
            (p.device, p.description)
            for p in serial.tools.list_ports.comports()
            if 'Arduino' in p.description  # Adjust this condition based on your Arduino model's description.
        ]
        return arduino_ports

    def Position_Only(self):
        if self.PositionOnly.isChecked():
            self.ui.FWHM_Plot.setEnabled(False)
            self.ui.Std_LA_Plot.setEnabled(False)
            self.ui.std_LA_text.setEnabled(False)
            self.ui.std_SA_text.setEnabled(False)
            self.ui.circularity_text.setEnabled(False)
        elif self.CurrentFrame > 0:
            self.ui.FWHM_Plot.setEnabled(True)
            self.ui.Std_LA_Plot.setEnabled(True)
            self.ui.std_LA_text.setEnabled(True)
            self.ui.std_SA_text.setEnabled(True)
            self.ui.circularity_text.setEnabled(True)


    def Auto_ExpTime(self):
        if self.AutoExpTime.isChecked():
            if self.CurrentFrame > 0:
                self.th.Auto_ExpTimeCond = True
            self.ExpTime_text.setEnabled(False)
        else:
            if self.CurrentFrame > 0:
                self.th.Auto_ExpTimeCond = False
            self.ExpTime_text.setEnabled(True)


    def StopAcquisition(self):
        self.th.Stop_Loop = True
        self.stop_acquisition.setEnabled(False)

    def CheckForInt_cam_serial(self, text):
        if not text.isdigit() and np.size(text) == 0:
            print(text)
            self.printed_info.setText('     Please Make sure all of the parameters are digits')
            self.cam_serial.setText('20270803')

    def CheckForInt_nr_of_frames(self, text):
        if not text.isdigit() and np.size(text) == 0:
            self.printed_info.setText('     Please Make sure all of the parameters are digits')
            self.nr_of_frames.setText('1000')

    def CheckForInt_pixel_size(self, text):
        if not text.isdigit() and np.size(text) == 0:
            self.printed_info.setText('     Please Make sure all of the parameters are digits')
            self.pixel_size.setText('0.0069')

    def CheckForInt_ExpTime_text(self, text):
        if not text.isdigit() and np.size(text) == 0:
            self.printed_info.setText('     Please Make sure all of the parameters are digits')
            self.ExpTime_text.setText('7000')

    def CheckForInt_FrameRate_text(self, text):
        if not text.isdigit() and np.size(text) == 0:
            self.printed_info.setText('     Please Make sure all of the parameters are digits')
            self.FrameRate_text.setText('50')

    def CheckForInt_FramesPerFile_text(self, text):
        if not text.isdigit() and np.size(text) == 0:
            self.printed_info.setText('     Please Make sure all of the parameters are digits')
            self.FramesPerFile_text.setText('1000')

    def Saving_Option(self):
        if self.SavingOption.isChecked():
            self.SavingLongAxis.setEnabled(True)
            self.DataFileName = QFileDialog.getSaveFileName(self, 'Save File', 'D:', "CSV Files (*.csv )")
            self.DataFileName = self.DataFileName[0]
            self.FramesPerFile_text.setEnabled(True)
            if self.DataFileName[len(self.DataFileName)-4: len(self.DataFileName)] != '.csv' and \
                    self.DataFileName[len(self.DataFileName)-4: len(self.DataFileName)] != '.CSV':
               self.DataFileName = self.DataFileName + '.csv'
        else:
            self.FramesPerFile_text.setEnabled(False)
            self.SavingLongAxis.setEnabled(False)
        self.printed_info.setText('     Please select the name of the csv file for the data.'
                                  ' Select the number of the data that will be saved in each file ')

    def PlotStdchange(self):
        self.StdPlot = pg.PlotWidget()
        self.StdPlot.setTitle("Std Value")
        self.StdPlot.plot(self.std2[0:self.CurrentFrame])
        self.StdPlot.setLabel(axis='left', text='Std/a.u.')
        self.StdPlot.setLabel(axis='bottom', text='Frame/a.u.')
        self.window_plot_Std = QMainWindow()
        self.window_plot_Std.setCentralWidget(self.StdPlot)
        self.window_plot_Std.show()

    def PlotFWHMchange(self):
        self.FWHMPlot = pg.PlotWidget()
        self.FWHMPlot.setTitle("FWHM Value")
        self.FWHMPlot.plot(self.FWHM[0:self.CurrentFrame])
        self.FWHMPlot.setLabel(axis='left', text='Width/mm')
        self.FWHMPlot.setLabel(axis='bottom', text='Frame/a.u.')
        self.window_plot_FWHM = QMainWindow()
        self.window_plot_FWHM.setCentralWidget(self.FWHMPlot)
        self.window_plot_FWHM.show()

    def PlotYchange(self):
        self.Y_Plot = pg.PlotWidget()
        self.Y_Plot.setTitle("Centroid Y-Position")
        self.Y_Plot.plot(self.Y_Max_Pos[0:self.CurrentFrame])
        self.Y_Plot.setLabel(axis='left', text='Position/mm')
        self.Y_Plot.setLabel(axis='bottom', text='Frame/a.u.')
        self.window_plot_y = QMainWindow()
        self.window_plot_y.setCentralWidget(self.Y_Plot)
        self.window_plot_y.show()

    def PlotXchange(self):
        self.X_Plot = pg.PlotWidget()
        self.X_Plot.setTitle("Centroid X-Position")
        self.X_Plot.plot(self.X_Max_Pos[0:self.CurrentFrame])
        self.X_Plot.setLabel(axis='left', text='Position/mm')
        self.X_Plot.setLabel(axis='bottom', text='Frame/a.u.')
        self.window_plot_x = QMainWindow()
        self.window_plot_x.setCentralWidget(self.X_Plot)
        self.window_plot_x.show()

    def PlotXchange_offline(self):
        self.X_Plot_offline = pg.PlotWidget()
        self.X_Plot_offline.setTitle("Centroid X-Position")
        self.X_Plot_offline.plot(self.X_Max_Pos_offline)
        self.window_plot_x_offline = QMainWindow()
        self.X_Plot_offline.setLabel(axis='left', text='Position/mm')
        self.X_Plot_offline.setLabel(axis='bottom', text='Frame/a.u.')
        self.window_plot_x_offline.setCentralWidget(self.X_Plot_offline)

        self.window_plot_x_offline.show()

    def PlotYchange_offline(self):
        self.Y_Plot_offline = pg.PlotWidget()
        self.Y_Plot_offline.setTitle("Centroid Y-Position")
        self.Y_Plot_offline.plot(self.Y_Max_Pos_offline)
        self.Y_Plot_offline.setLabel(axis='left', text='Position/mm')
        self.Y_Plot_offline.setLabel(axis='bottom', text='Frame/a.u.')
        self.window_plot_y_offline = QMainWindow()
        self.window_plot_y_offline.setCentralWidget(self.Y_Plot_offline)
        self.window_plot_y_offline.show()


    def OpenFile(self):
        self.allfiles = QFileDialog.getOpenFileNames(self, 'Open file', 'D:')
        self.allfiles = self.allfiles[0]
        self.Nr_allfiles = len(self.allfiles)
        self.ui_offline.FrameSlider.setMaximum(self.Nr_allfiles-1)
        self.X_Max_Pos_offline = np.zeros(self.Nr_allfiles)  # array to follow the X-position of the beam center
        self.Y_Max_Pos_offline = np.zeros(self.Nr_allfiles)  # array to follow the Y-position of the beam center
        self.Circ_offline = np.zeros(self.Nr_allfiles)

        for i in range (self.Nr_allfiles):

            im = skimage.io.imread(self.allfiles[i])
            im = (im - np.min(im)) / (np.max(im) - np.min(im))  # normalize the matrix
            upper_limit = 0.505
            lower_limit = 0.495
            cond = True  # makes sure the elipse is detected
            while cond:
                yx_coords = np.column_stack(np.where((im >= lower_limit) & (im <= upper_limit)))
                if np.max(np.shape(yx_coords)) > 2:
                    upper_limit = 0.505
                    lower_limit = 0.495
                    cond = False
                else:
                    upper_limit = upper_limit + 0.005
                    lower_limit = lower_limit - 0.005
                if (upper_limit > 1) or (lower_limit < 0):
                    print(f'Frame number {i} was skipped')
                    i = i+1
                    im = skimage.io.imread(self.allfiles[i])
                    upper_limit = 0.505
                    lower_limit = 0.495
                if i >= self.Nr_allfiles - 1:
                    break

            Y_Center = ((np.amax(yx_coords[:, 0]) + np.amin(yx_coords[:, 0])) / 2)
            X_Center = ((np.amax(yx_coords[:, 1]) + np.amin(yx_coords[:, 1])) / 2)
            d_from_center = (yx_coords[:, 1] - X_Center) ** 2 + (yx_coords[:, 0] - Y_Center) ** 2
            self.X_Max_Pos_offline[i] = X_Center * float(self.pixel_size.text())
            self.Y_Max_Pos_offline[i] = Y_Center * float(self.pixel_size.text())
            self.Circ_offline[i] = math.sqrt(np.min(d_from_center)) / math.sqrt(np.max(d_from_center))
        self.ui_offline.FrameSlider.valueChanged.connect(self.ChangViewedFrame)
        self.ui_offline.X_Pos_Plot_offline.clicked.connect(self.PlotXchange_offline)
        self.ui_offline.Y_Pos_Plot_offline.clicked.connect(self.PlotYchange_offline)

        self.window2.show()

    def ChangViewedFrame(self, value):

        im = self.gray2qimage(skimage.io.imread(self.allfiles[value]))
        self.ui_offline.Frame_view.setPixmap(QtGui.QPixmap(QtGui.QPixmap.fromImage(im)))
        self.ui_offline.FrameNr.setText(str(value))
        self.ui_offline.circularity_text_offline.setText(str(self.Circ_offline[value]))
        self.ui_offline.x_position_text_offline.setText(str(self.X_Max_Pos_offline[value]))
        self.ui_offline.y_position_text_offline.setText(str(self.Y_Max_Pos_offline[value]))




    def cam_serial_info(self):
        self.printed_info.setText('     Please enter the serial of the FLIR camera you are using.')

    def nr_of_frames_info(self):
        self.printed_info.setText('     Please enter the number of frames of the recording.')

    def PrintAnalysisInfo(self):
        self.printed_info.setText(f'     This GUI processes consecutive frames of a laser beam. The elliptic shape of'
                                  f' the beam is detected by selecting the pixels with values within an intensity range '
                                  f'(between 49.5% and 50.5%). The center of this ellipse is detected and the point with'
                                  f' the largest distance from the center will represent the long axis of the ellipse. '
                                  f'\n A'
                                  f' Gaussian-fit is used to get the Std of the beam and the FWHM is calculated directly'
                                  f' from the frame. The user can upload images to the GUI or stream a video from a FLIR'
                                  f' camera.')

    def stream_type_info(self):
        self.printed_info.setText(f'   Please select Type of the data streamed, either a life stream from a FLIR cam, '
                                  f'or images from previous recording')

    def PrintAboutInfo(self):
        self.printed_info.setText(f'  Name: BeamProfiler \n'
                                  f'  Version: 1.0 \n'
                                  f'  Author: Mohamed Shehata\n'
                                  f'  Supervision: Ph.D. Tobias Önol-Nöbauer \n'
                                  f'  16.06.2023 The Rockefeller University\n'
                                  f'  New York, USA \n')

    def frame_rate_info(self):
        self.printed_info.setText('     Please enter the desired frame rate (Max frame rate is 33 per second)')

    def pixel_size_info(self):
        self.printed_info.setText('     Please enter the pixel size of the camera you are using in mm.')

    def exp_time_info(self):
        self.printed_info.setText('     Please enter the Exposure time of the camera you are using in micro-seconds.')

    def OpenViewer(self):

        self.window1.show()

    @QtCore.pyqtSlot(np.ndarray, int, int)
    def ShowFrame(self, FrameNP, i, TimeStamp):
        FramePIL = PIL.Image.fromarray(FrameNP)  # get the result as a PIL image

        self.ui.cam_view.setPixmap(QtGui.QPixmap(QtGui.QPixmap.fromImage(self.gray2qimage(FrameNP))))
        FrameNP = (FrameNP - np.min(FrameNP)) / (np.max(FrameNP) - np.min(FrameNP))  # normalize the matrix
        # the elliptic shape of the beam is extracted on the form of points on a certain intensity of the gaussian beam
        upper_limit = 0.505
        lower_limit = 0.495
        cond = True # makes sure the elipse is detected
        while cond:
            yx_coords = np.column_stack(np.where((FrameNP >= lower_limit) & (FrameNP <= upper_limit)))
            if np.max(np.shape(yx_coords)) > 2:
                upper_limit = 0.505
                lower_limit = 0.495
                cond = False
            else:
                upper_limit = upper_limit + 0.005
                lower_limit = lower_limit - 0.005
            if (upper_limit > 1) or (lower_limit < 0):
                print('Make sure camera is open')
                cond = False

        Y_Center = ((np.amax(yx_coords[:, 0]) + np.amin(yx_coords[:, 0])) / 2)
        X_Center = ((np.amax(yx_coords[:, 1]) + np.amin(yx_coords[:, 1])) / 2)
        # the axis is set at the furthest point from the center
        d_from_center = (yx_coords[:, 1] - X_Center) ** 2 + (yx_coords[:, 0] - Y_Center) ** 2
        P_axis = np.where(d_from_center == np.amax(d_from_center))
        P_axis = yx_coords[P_axis[0]]

        m = (P_axis[0][0] - Y_Center) / (P_axis[0][1] - X_Center)  # the slope of the axis

        # centering and croping the image
        FramePIL = FramePIL.crop(
            (round(X_Center - 190), round(Y_Center - 190), round(X_Center + 190), round(Y_Center + 190)))
        FramePIL = FramePIL.rotate(abs(math.atan(m) * 180 / math.pi) + 90)
        # obtain a cross-section of the gaussian beam
        Height = FramePIL.height
        Width = FramePIL.width
        FramePIL_col = FramePIL.crop(
            (round((Width/2) - 3), 0, round((Width/2) + 3), Height)) # 6 pixels width
        sum_rows = np.mean(FramePIL, axis=0)  # get array with the sums of the rows
        sum_col = np.mean(FramePIL_col, axis=1)  # get array with the sums of the columns

        x_col = np.linspace(0, FramePIL.height - 1, num=FramePIL.height) * float(self.pixel_size.text())  # dim in mm
        x_rows = np.linspace(0, FramePIL.width - 1, num=FramePIL.width) * float(self.pixel_size.text())  # dim in mm
        sum_col = (sum_col - np.min(sum_col)) / (np.max(sum_col) - np.min(sum_col))
        cond = True  # makes sure the elipse is detected
        while cond:
            sum_col_05 = (np.where((sum_col >= lower_limit) & (sum_col <= upper_limit)))
            if np.max(np.shape(sum_col_05)) > 1:
                self.FWHM[self.CurrentFrame] = abs(
                    x_col[sum_col_05[0][0]] - x_col[sum_col_05[0][np.max(np.shape(sum_col_05)) - 1]])

                if self.FWHM[self.CurrentFrame] <= 5 * float(self.pixel_size.text()):
                    upper_limit = upper_limit + 0.005
                    lower_limit = lower_limit - 0.005
                else:
                    upper_limit = 0.505
                    lower_limit = 0.495
                    cond = False
            else:
                upper_limit = upper_limit + 0.005
                lower_limit = lower_limit - 0.005
            if (upper_limit > 1) or (lower_limit < 0):
                print('Make sure camera is open')
                cond = False



        # curve fitting
        popt1, pcov1 = sp.optimize.curve_fit(self.gauss, x_rows, sum_rows)

        popt2, pcov2 = sp.optimize.curve_fit(self.gauss, x_col, sum_col)

        self.CurrentFrame = i - self.SavedFileNumber * int(self.FramesPerFile_text.text())
        self.ui.circularity_text.setText(str( math.sqrt(np.min(d_from_center)) / math.sqrt(np.max(d_from_center))))
        self.ui.std_LA_text.setText(str(abs(popt2[2])))
        self.ui.std_SA_text.setText(str(abs(popt1[2])))
        #self.FWHM[self.CurrentFrame] = 2 * math.sqrt(np.max(d_from_center)) * float(self.pixel_size.text())
        self.X_Max_Pos[self.CurrentFrame] = X_Center * float(self.pixel_size.text())
        self.Y_Max_Pos[self.CurrentFrame] = Y_Center * float(self.pixel_size.text())
        self.LongAxisPlot[self.CurrentFrame, :] = sum_col
        self.std2[self.CurrentFrame] = abs(popt2[2])
        self.FrameTime[self.CurrentFrame] = TimeStamp
        self.ui.x_position_text.setText(str(self.X_Max_Pos[self.CurrentFrame]))
        self.ui.y_position_text.setText(str(self.Y_Max_Pos[self.CurrentFrame]))
        self.Plot_Gauss.plot(x_col, sum_col, clear=True)

        if self.SavingOption.isChecked() and ((self.CurrentFrame == int(self.nr_of_frames.text())) or
                                               (self.CurrentFrame == self.FileStreamNr-1) or
                                               (self.CurrentFrame == int(self.FramesPerFile_text.text()) - 1) or
                                               self.RemainingFrames == int(self.nr_of_frames.text()) -
                                               int(self.FramesPerFile_text.text()) * self.SavedFileNumber):

            if (self.CurrentFrame == int(self.FramesPerFile_text.text()) - 1) and \
                    (int(self.nr_of_frames.text()) > int(self.FramesPerFile_text.text())):
                np.savetxt(self.DataFileName[0: len(self.DataFileName)-4] + str(self.SavedFileNumber) + '.csv',
                           [self.FrameTime ,self.X_Max_Pos, self.Y_Max_Pos, self.std2, self.FWHM],
                           delimiter=",")
                if self.SavingLongAxis.isChecked:
                    np.savetxt(self.DataFileName[0: len(self.DataFileName) - 4] + 'Long axis plot' +
                               str(self.SavedFileNumber) + '.csv', self.LongAxisPlot, delimiter=",")
                self.SavedFileNumber = self.SavedFileNumber + 1
                self.RemainingFrames = self.RemainingFrames - int(self.FramesPerFile_text.text())
                self.X_Max_Pos = np.zeros(
                    int(self.FramesPerFile_text.text()))  # array to follow the X-position of the beam center
                self.Y_Max_Pos = np.zeros(
                    int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center
                self.pos2 = np.zeros(int(self.FramesPerFile_text.text()))
                self.std2 = np.zeros(int(self.FramesPerFile_text.text()))
                self.FWHM = np.zeros(int(self.FramesPerFile_text.text()))
                self.LongAxisPlot = np.zeros((int(self.FramesPerFile_text.text()), 380))
            elif (self.RemainingFrames > 0) and (self.RemainingFrames < int(self.FramesPerFile_text.text())) and \
                    self.RemainingFrames == int(self.nr_of_frames.text()) - \
                    int(self.FramesPerFile_text.text()) * self.SavedFileNumber:
                self.stop_acquisition.setEnabled(False)
                np.savetxt(self.DataFileName[0: len(self.DataFileName)-4] + str(self.SavedFileNumber) + '.csv',
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos, self.std2, self.FWHM],
                           delimiter=",")

                if self.SavingLongAxis.isChecked:
                    np.savetxt(self.DataFileName[0: len(self.DataFileName) - 4] + 'Long axis plot' +
                               str(self.SavedFileNumber) + '.csv', self.LongAxisPlot, delimiter=",")
            elif self.CurrentFrame == int(self.nr_of_frames.text()) - 1:
                self.stop_acquisition.setEnabled(False)
                np.savetxt(self.DataFileName,
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos, self.std2, self.FWHM],
                           delimiter=",")
                if self.SavingLongAxis.isChecked:
                    np.savetxt(self.DataFileName[0: len(self.DataFileName) - 4] + 'Long axis plot'+
                               str(self.SavedFileNumber) + '.csv', self.LongAxisPlot, delimiter=",")

    def ShowFrame_PositionOnly(self, FrameNP, i, TimeStamp):



        self.ui.cam_view.setPixmap(QtGui.QPixmap(QtGui.QPixmap.fromImage(self.gray2qimage(FrameNP))))
        FrameNP = (FrameNP - np.min(FrameNP)) / (np.max(FrameNP) - np.min(FrameNP))  # normalize the matrix
        # the elliptic shape of the beam is extracted on the form of points on a certain intensity of the gaussian beam
        upper_limit = 0.505
        lower_limit = 0.495
        cond = True  # makes sure the elipse is detected
        while cond:
            yx_coords = np.column_stack(np.where((FrameNP >= lower_limit) & (FrameNP <= upper_limit)))
            if np.max(np.shape(yx_coords)) > 2:
                upper_limit = 0.505
                lower_limit = 0.495
                cond = False
            else:
                upper_limit = upper_limit + 0.005
                lower_limit = lower_limit - 0.005
            if (upper_limit > 1) or (lower_limit < 0):
                print('Make sure camera is open')
                cond = False
        self.CurrentFrame = i - self.SavedFileNumber * int(self.FramesPerFile_text.text())
        Y_Center = ((np.amax(yx_coords[:, 0]) + np.amin(yx_coords[:, 0])) / 2)
        X_Center = ((np.amax(yx_coords[:, 1]) + np.amin(yx_coords[:, 1])) / 2)
        self.X_Max_Pos[self.CurrentFrame] = X_Center * float(self.pixel_size.text())
        self.Y_Max_Pos[self.CurrentFrame] = Y_Center * float(self.pixel_size.text())
        self.FrameTime[self.CurrentFrame] = TimeStamp
        self.ui.x_position_text.setText(str(self.X_Max_Pos[self.CurrentFrame]))
        self.ui.y_position_text.setText(str(self.Y_Max_Pos[self.CurrentFrame]))
        if self.SavingOption.isChecked() and ((self.CurrentFrame == int(self.nr_of_frames.text())) or
                                              (self.CurrentFrame == self.FileStreamNr - 1) or
                                              (self.CurrentFrame == int(self.FramesPerFile_text.text()) - 1) or
                                              self.RemainingFrames == int(self.nr_of_frames.text()) -
                                              int(self.FramesPerFile_text.text()) * self.SavedFileNumber):

            if (self.CurrentFrame == int(self.FramesPerFile_text.text()) - 1) and \
                    (int(self.nr_of_frames.text()) > int(self.FramesPerFile_text.text())):
                np.savetxt(self.DataFileName[0: len(self.DataFileName) - 4] + str(self.SavedFileNumber) + '.csv',
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos],
                           delimiter=",")

                self.SavedFileNumber = self.SavedFileNumber + 1
                self.RemainingFrames = self.RemainingFrames - int(self.FramesPerFile_text.text())
                self.X_Max_Pos = np.zeros(
                    int(self.FramesPerFile_text.text()))  # array to follow the X-position of the beam center
                self.Y_Max_Pos = np.zeros(
                    int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center

            elif (self.RemainingFrames > 0) and (self.RemainingFrames < int(self.FramesPerFile_text.text())) and \
                    self.RemainingFrames == int(self.nr_of_frames.text()) - \
                    int(self.FramesPerFile_text.text()) * self.SavedFileNumber:
                self.stop_acquisition.setEnabled(False)
                np.savetxt(self.DataFileName[0: len(self.DataFileName) - 4] + str(self.SavedFileNumber) + '.csv',
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos],
                           delimiter=",")


            elif self.CurrentFrame == int(self.nr_of_frames.text()) - 1:
                self.stop_acquisition.setEnabled(False)
                np.savetxt(self.DataFileName,
                           [self.FrameTime, self.X_Max_Pos, self.Y_Max_Pos],
                           delimiter=",")

    def StartAcquisition(self):
        self.stop_acquisition.setEnabled(True)
        self.CurrentFrame = 0
        self.SavedFileNumber = 0
        self.Plot_Gauss = pg.PlotWidget()  # adding a plot to show the gaussian dist. of the long axis
        self.Plot_Gauss.setTitle("Gaussian Cross Section.")
        self.Plot_Gauss.setLabel(axis='left', text='Average Intensity/a.u.')
        self.Plot_Gauss.setLabel(axis='bottom', text='Distance/mm')
        self.window_Plot_Gauss = QMainWindow()
        self.window_Plot_Gauss.setCentralWidget(self.Plot_Gauss)
        self.window_Plot_Gauss.show()
        self.printed_info.setText('   Acquisition starting... Please press Open Viewer to see the camera video stream.')
        self.StartingTime = time.time()
        if self.StreamType.currentText() == "FLIR Cam":
            self.th = Thread_FLIR(self)  # calling the thread of the camera
            if self.PositionOnly.isChecked():
                self.th.changePixmap_FLIR.connect(self.ShowFrame_PositionOnly)
                self.window_Plot_Gauss.close()
            else:
                self.th.changePixmap_FLIR.connect(self.ShowFrame)
            self.th.NUM_IMG = int(self.nr_of_frames.text())
            self.th.serial = (self.cam_serial.text())
            self.th.ExpTime_us = float(self.ExpTime_text.text())
            self.th.frame_rate = float(self.FrameRate_text.text())
            self.th.Auto_ExpTimeCond = self.AutoExpTime.isChecked()
            self.FileStreamNr = int(self.nr_of_frames.text())
            if int(self.nr_of_frames.text()) > int(self.FramesPerFile_text.text()) and self.SavingOption.isChecked():
                if self.PositionOnly.isChecked():
                    self.X_Max_Pos = np.zeros(
                       int(self.FramesPerFile_text.text()))  # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(
                       int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center
                    self.FrameTime = np.zeros(int(self.FramesPerFile_text.text()))
                else:
                    self.X_Max_Pos = np.zeros(
                        int(self.FramesPerFile_text.text()))  # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(
                        int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center
                    self.pos2 = np.zeros(int(self.FramesPerFile_text.text()))
                    self.std2 = np.zeros(int(self.FramesPerFile_text.text()))
                    self.FWHM = np.zeros(int(self.FramesPerFile_text.text()))
                    self.FrameTime = np.zeros(int(self.FramesPerFile_text.text()))
                    self.LongAxisPlot = np.zeros((int(self.FramesPerFile_text.text()), 380))
            else:
                if self.PositionOnly.isChecked():
                    self.X_Max_Pos = np.zeros(
                        int(self.nr_of_frames.text()))  # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(
                        int(self.nr_of_frames.text()))  # array to follow the Y-position of the beam center
                    self.FrameTime = np.zeros(int(self.nr_of_frames.text()))
                else:
                    self.X_Max_Pos = np.zeros(
                        int(self.nr_of_frames.text()))  # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(
                        int(self.nr_of_frames.text()))  # array to follow the Y-position of the beam center
                    self.pos2 = np.zeros(int(self.nr_of_frames.text()))
                    self.std2 = np.zeros(int(self.nr_of_frames.text()))
                    self.FWHM = np.zeros(int(self.nr_of_frames.text()))
                    self.FrameTime = np.zeros(int(self.nr_of_frames.text()))
                    self.LongAxisPlot = np.zeros((int(self.nr_of_frames.text()), 380))
            self.th.start()

        elif self.StreamType.currentText() == "Data Stream":
            self.th = Thread_Data(self)  # calling the thread of the camera
            if self.PositionOnly.isChecked():
                self.th.changePixmap_Data.connect(self.ShowFrame_PositionOnly)
            else:
                self.th.changePixmap_Data.connect(self.ShowFrame)
            self.th.NUM_IMG = int(self.nr_of_frames.text())
            self.th.serial = (self.cam_serial.text())
            self.th.ExpTime_us = float(self.ExpTime_text.text())
            self.th.frame_rate = float(self.FrameRate_text.text())
            self.th.file_names = QFileDialog.getOpenFileNames(self, 'Open file', 'D:')
            self.th.file_names = self.th.file_names[0]
            self.FileStreamNr = len(self.th.file_names)
            self.nr_of_frames.setText(str(len(self.th.file_names)))

            if self.SavingOption.isChecked() and int(self.FramesPerFile_text.text()) < self.FileStreamNr:
                if self.PositionOnly.isChecked():
                    self.X_Max_Pos = np.zeros(int(self.FramesPerFile_text.text())) # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center
                    self.FrameTime = np.zeros(int(self.FramesPerFile_text.text()))
                else:
                    self.X_Max_Pos = np.zeros(
                        int(self.FramesPerFile_text.text()))  # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(
                        int(self.FramesPerFile_text.text()))  # array to follow the Y-position of the beam center
                    self.pos2 = np.zeros(int(self.FramesPerFile_text.text()))
                    self.std2 = np.zeros(int(self.FramesPerFile_text.text()))
                    self.FWHM = np.zeros(int(self.FramesPerFile_text.text()))
                    self.LongAxisPlot = np.zeros((int(self.FramesPerFile_text.text()), 380))
                    self.FrameTime = np.zeros(int(self.FramesPerFile_text.text()))
            else:
                if self.PositionOnly.isChecked():
                    self.X_Max_Pos = np.zeros(self.FileStreamNr)  # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(self.FileStreamNr)  # array to follow the Y-position of the beam center
                    self.FrameTime = np.zeros(self.FileStreamNr)
                else:
                    self.X_Max_Pos = np.zeros(self.FileStreamNr)  # array to follow the X-position of the beam center
                    self.Y_Max_Pos = np.zeros(self.FileStreamNr)  # array to follow the Y-position of the beam center
                    self.pos2 = np.zeros(self.FileStreamNr)
                    self.std2 = np.zeros(self.FileStreamNr)
                    self.FWHM = np.zeros(self.FileStreamNr)
                    self.LongAxisPlot = np.zeros((self.FileStreamNr, 380))
                    self.FrameTime = np.zeros(self.FileStreamNr)
            self.th.start()
        self.RemainingFrames = int(self.nr_of_frames.text())

    def gauss(self, x, A, mu, sigma, off):
        return A * np.exp(-(x - mu) ** 2 / (2 * sigma ** 2)) + off

    def rotate_stepper(self, steps, direction, board, ENA_PIN, DIR_PIN, PUL_PIN, OPTO_Pin):
        board.digital[OPTO_Pin].write(1)  # Enable the stepper driver.
        time.sleep(0.0005)
        board.digital[ENA_PIN].write(1)  # Enable the stepper driver.
        time.sleep(0.0005)

        # Set the direction (HIGH for clockwise, LOW for counterclockwise).
        board.digital[DIR_PIN].write(direction)
        time.sleep(0.0005)

        for _ in range(steps):
            board.digital[PUL_PIN].write(1)  # Send a step signal.
            time.sleep(0.000075)
            board.digital[PUL_PIN].write(0)
            time.sleep(0.000075)

        board.digital[ENA_PIN].write(0)  # Disable the stepper driver.
        board.digital[OPTO_Pin].write(0)  # Disable the stepper driver.

    def rotate_stepper_continuous_X(self, direction, board, ENA_PIN, DIR_PIN, PUL_PIN, OPTO_Pin):
        board.digital[OPTO_Pin].write(1)  # Enable the stepper driver.
        time.sleep(0.0005)
        board.digital[ENA_PIN].write(1)  # Enable the stepper driver.
        time.sleep(0.0005)

        # Set the direction (HIGH for clockwise, LOW for counterclockwise).
        board.digital[DIR_PIN].write(direction)
        time.sleep(0.0005)

        while True:
            if self.cond_motor_x:
                break
            board.digital[PUL_PIN].write(1)  # Send a step signal.
            time.sleep(0.000075)
            board.digital[PUL_PIN].write(0)
            time.sleep(0.000075)


        board.digital[ENA_PIN].write(0)  # Disable the stepper driver.
        board.digital[OPTO_Pin].write(0)  # Disable the stepper driver.

    def rotate_stepper_continuous_Y(self, direction, board, ENA_PIN, DIR_PIN, PUL_PIN, OPTO_Pin):
        board.digital[OPTO_Pin].write(1)  # Enable the stepper driver.
        time.sleep(0.0005)
        board.digital[ENA_PIN].write(1)  # Enable the stepper driver.
        time.sleep(0.0005)

        # Set the direction (HIGH for clockwise, LOW for counterclockwise).
        board.digital[DIR_PIN].write(direction)
        time.sleep(0.0005)

        while True:
            if self.cond_motor_y:
                break
            board.digital[PUL_PIN].write(1)  # Send a step signal.
            time.sleep(0.000075)
            board.digital[PUL_PIN].write(0)
            time.sleep(0.000075)

        board.digital[ENA_PIN].write(0)  # Disable the stepper driver.
        board.digital[OPTO_Pin].write(0)  # Disable the stepper driver.

    def gray2qimage(self, gray):
        '''
        Convert numpy array to QtImage
        '''
        if len(gray.shape) != 2:
            raise ValueError("gray2QImage can only convert 2D arrays")

        h, w = gray.shape
        if gray.dtype == np.uint8:
            gray = np.require(gray, np.uint8, 'C')
            result = QtGui.QImage(gray.data, w, h, QtGui.QImage.Format_Indexed8)
            result.ndarray = gray
            for i in range(256):
                result.setColor(i, QtGui.QColor(i, i, i).rgb())
        elif gray.dtype == np.uint16:
            # This assumes that a 16bit image is only 12bit resolution: 2^16-2^12=16
            gray = (gray / 256).astype(np.uint8)
            gray = np.require(gray, np.uint8, 'C')
            h, w = gray.shape
            result = QtGui.QImage(gray.data, w, h, QtGui.QImage.Format_Indexed8)
            result.ndarray = gray
            # for i in range(256):
            #    result.setColor(i, QtGui.QColor(i, i, i).rgb())
        else:
            # Convert by multiplying by 256 and making it uint8
            # print gray * 2**(8-12)
            # gray = (gray * 256).astype(numpy.uint8)
            gray = (gray * 2 ** (8 - 12)).astype(np.uint8)
            gray = np.require(gray, np.uint8, 'C')
            h, w = gray.shape
            result = QtGui.QImage(gray.data, w, h, QtGui.QImage.Format_Indexed8)
            result.ndarray = gray
            for i in range(256):
                result.setColor(i, QtGui.QColor(i, i, i).rgb())
        return result


# if __name__ == '__main__':
app = QApplication([])
window = PyBeamProfilerGUI()
app.exec_()
