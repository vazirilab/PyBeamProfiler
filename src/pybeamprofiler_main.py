from PyQt5.QtWidgets import *
from PyQt5 import uic
# from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import QtGui, QtCore
import skimage
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
import PIL
# from PIL import ImageFilter, Image
import math
import time
import scipy as sp
import PySpin
import cv2
from View_page import Ui_MainWindow
import pyqtgraph as pg


# A thread for the camera. It emits frames and the number of the current frame
class Thread(QtCore.QThread):
    changePixmap = QtCore.pyqtSignal(np.ndarray, int)
    NUM_IMG = 100000
    serial = '20270803'  # serisl of the flir cam
    ppmm = 0.0069  # pixels per mm to convert position to mm
    ppmm = 1 / ppmm
    system = PySpin.System.GetInstance()  # Retrieve singleton reference to system object
    version = system.GetLibraryVersion()  # Get current library version
    ExpTime_us = 8000
    cam_list = system.GetCameras()  # Retrieve list of cameras from the system

    cam = cam_list.GetBySerial(serial)
    cam.Init()
    processor = PySpin.ImageProcessor()
    cam.BeginAcquisition()

    def run(self):
        self.cam.ExposureTime.SetValue(self.ExpTime_us)
        for i in range(self.NUM_IMG):
            # gg = np.random.rand(200, 200) * 1155
            # height, width = gg.shape
            image_result = self.cam.GetNextImage(1000)
            FrameNP = image_result.GetNDArray()  # get the result as a numpy array

            time.sleep(0.01)
            self.changePixmap.emit(FrameNP, i)


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
        self.actionOpen.triggered.connect(self.OpenFile)
        self.cam_serial.setText('20270803')
        self.nr_of_frames.setText('1000')
        self.pixel_size.setText('0.0069')
        self.ExpTime_text.setText('7000')

        self.ui = Ui_MainWindow()
        self.ui.X_Pos_Plot.clicked.connect(self.PlotXchange)
        self.ui.Y_Pos_Plot.clicked.connect(self.PlotYchange)

    def PlotYchange(self):
        self.Y_Plot = pg.PlotWidget()
        self.X_Plot.setTitle("Y Position")
        self.Y_Plot.plot(self.Y_Max_Pos[0:self.CurrentFrame])
        self.setCentralWidget(self.Y_Plot)

    def PlotXchange(self):
        self.X_Plot = pg.PlotWidget()
        self.X_Plot.setTitle("X Position")
        self.X_Plot.plot(self.X_Max_Pos[0:self.CurrentFrame])
        self.setCentralWidget(self.X_Plot)

    def OpenFile(self):
        print(1)

    def cam_serial_info(self):
        self.printed_info.setText('     Please enter the serial of the FLIR camera you are using. In case you do not '
                                  'enter anything or a correct serial number, the program will select the first '
                                  'camera on the index.')

    def nr_of_frames_info(self):
        self.printed_info.setText('     Please enter the number of frames of the recording.')

    def pixel_size_info(self):
        self.printed_info.setText('     Please enter the pixel size of the camera you are using.')

    def OpenViewer(self):
        self.window1 = QMainWindow()
        self.ui.setupUi(self.window1)

        self.window1.show()

    @QtCore.pyqtSlot(np.ndarray, int)
    def ShowFrame(self, FrameNP, i):
        FramePIL = PIL.Image.fromarray(FrameNP)  # get the result as a PIL image

        self.ui.std_LA_text.setText(str(np.max(FrameNP)))
        self.ui.cam_view.setPixmap(QtGui.QPixmap(QtGui.QPixmap.fromImage(self.gray2qimage(FrameNP))))
        FrameNP = (FrameNP) / (np.max(FrameNP))  # normalize the matrix
        # the elliptic shape of the beam is extracted on the form of points on a certain intensity of the gaussian beam
        yx_coords = np.column_stack(np.where((FrameNP >= 0.895) & (FrameNP <= 0.905)))
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
        sum_rows = np.mean(FramePIL, axis=0)  # get array with the sums of the rows
        sum_col = np.mean(FramePIL, axis=1)  # get array with the sums of the columns

        x_col = np.linspace(0, FramePIL.height - 1, num=FramePIL.height) * float(self.pixel_size.text())  # dim in mm
        x_rows = np.linspace(0, FramePIL.width - 1, num=FramePIL.width) * float(self.pixel_size.text())  # dim in mm

        popt1, pcov1 = sp.optimize.curve_fit(self.gauss, x_rows, sum_rows)

        popt2, pcov2 = sp.optimize.curve_fit(self.gauss, x_col, sum_col)

        self.CurrentFrame = i
        self.ui.circularity_text.setText(str(np.min(d_from_center) / np.max(d_from_center)))
        self.ui.std_LA_text.setText(str(abs(popt2[2])))
        self.ui.std_SA_text.setText(str(abs(popt1[2])))
        self.X_Max_Pos[i] = X_Center * float(self.pixel_size.text())
        self.Y_Max_Pos[i] = Y_Center * float(self.pixel_size.text())
        self.ui.x_position_text.setText(str(X_Center))
        self.ui.y_position_text.setText(str(Y_Center))

    def StartAcquisition(self):
        self.X_Max_Pos = np.zeros(int(self.nr_of_frames.text()))  # array to follow the X-position of the beam center
        self.Y_Max_Pos = np.zeros(int(self.nr_of_frames.text()))  # array to follow the Y-position of the beam center
        self.pos2 = np.zeros(int(self.nr_of_frames.text()))
        self.std2 = np.zeros(int(self.nr_of_frames.text()))
        self.FWHM = np.zeros(int(self.nr_of_frames.text()))
        th = Thread(self)
        th.changePixmap.connect(self.ShowFrame)
        th.NUM_IMG = int(self.nr_of_frames.text())
        th.serial = (self.cam_serial.text())
        th.ExpTime_us = float(self.ExpTime_text.text())
        th.start()

    def gauss(self, x, A, mu, sigma, off):
        return A * np.exp(-(x - mu) ** 2 / (2 * sigma ** 2)) + off

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
            gray = (gray / 16).astype(np.uint8)
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
