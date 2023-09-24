# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'View_page.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1029, 707)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setEnabled(False)
        self.lineEdit.setGeometry(QtCore.QRect(60, 10, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.lineEdit.setFont(font)
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setEnabled(False)
        self.lineEdit_2.setGeometry(QtCore.QRect(600, 10, 101, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.lineEdit_2.setFont(font)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_5 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_5.setEnabled(False)
        self.lineEdit_5.setGeometry(QtCore.QRect(10, 50, 91, 21))
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.lineEdit_6 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_6.setEnabled(False)
        self.lineEdit_6.setGeometry(QtCore.QRect(10, 80, 91, 21))
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.lineEdit_7 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_7.setEnabled(False)
        self.lineEdit_7.setGeometry(QtCore.QRect(10, 110, 91, 21))
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.lineEdit_8 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_8.setEnabled(False)
        self.lineEdit_8.setGeometry(QtCore.QRect(10, 140, 91, 21))
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.Circularity = QtWidgets.QLineEdit(self.centralwidget)
        self.Circularity.setEnabled(False)
        self.Circularity.setGeometry(QtCore.QRect(10, 170, 91, 21))
        self.Circularity.setObjectName("Circularity")
        self.x_position_text = QtWidgets.QLineEdit(self.centralwidget)
        self.x_position_text.setGeometry(QtCore.QRect(90, 50, 161, 21))
        self.x_position_text.setObjectName("x_position_text")
        self.y_position_text = QtWidgets.QLineEdit(self.centralwidget)
        self.y_position_text.setGeometry(QtCore.QRect(90, 80, 161, 21))
        self.y_position_text.setObjectName("y_position_text")
        self.std_LA_text = QtWidgets.QLineEdit(self.centralwidget)
        self.std_LA_text.setGeometry(QtCore.QRect(90, 110, 161, 21))
        self.std_LA_text.setObjectName("std_LA_text")
        self.std_SA_text = QtWidgets.QLineEdit(self.centralwidget)
        self.std_SA_text.setGeometry(QtCore.QRect(90, 140, 161, 21))
        self.std_SA_text.setObjectName("std_SA_text")
        self.circularity_text = QtWidgets.QLineEdit(self.centralwidget)
        self.circularity_text.setGeometry(QtCore.QRect(90, 170, 161, 21))
        self.circularity_text.setObjectName("circularity_text")
        self.cam_view = QtWidgets.QLabel(self.centralwidget)
        self.cam_view.setGeometry(QtCore.QRect(290, 60, 721, 521))
        self.cam_view.setText("")
        self.cam_view.setScaledContents(True)
        self.cam_view.setAlignment(QtCore.Qt.AlignCenter)
        self.cam_view.setObjectName("cam_view")
        self.X_Pos_Plot = QtWidgets.QPushButton(self.centralwidget)
        self.X_Pos_Plot.setGeometry(QtCore.QRect(10, 200, 171, 41))
        self.X_Pos_Plot.setObjectName("X_Pos_Plot")
        self.Y_Pos_Plot = QtWidgets.QPushButton(self.centralwidget)
        self.Y_Pos_Plot.setGeometry(QtCore.QRect(10, 250, 171, 41))
        self.Y_Pos_Plot.setObjectName("Y_Pos_Plot")
        self.FWHM_Plot = QtWidgets.QPushButton(self.centralwidget)
        self.FWHM_Plot.setGeometry(QtCore.QRect(10, 300, 171, 41))
        self.FWHM_Plot.setObjectName("FWHM_Plot")
        self.Std_LA_Plot = QtWidgets.QPushButton(self.centralwidget)
        self.Std_LA_Plot.setGeometry(QtCore.QRect(10, 350, 171, 41))
        self.Std_LA_Plot.setObjectName("Std_LA_Plot")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(250, 50, 31, 21))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(250, 80, 31, 21))
        self.label_2.setObjectName("label_2")
        self.live_view_x = QtWidgets.QCheckBox(self.centralwidget)
        self.live_view_x.setGeometry(QtCore.QRect(190, 210, 71, 21))
        self.live_view_x.setObjectName("live_view_x")
        self.live_view_y = QtWidgets.QCheckBox(self.centralwidget)
        self.live_view_y.setGeometry(QtCore.QRect(190, 260, 71, 21))
        self.live_view_y.setObjectName("live_view_y")
        self.live_view_fwhm = QtWidgets.QCheckBox(self.centralwidget)
        self.live_view_fwhm.setGeometry(QtCore.QRect(190, 310, 71, 21))
        self.live_view_fwhm.setObjectName("live_view_fwhm")
        self.live_view_std = QtWidgets.QCheckBox(self.centralwidget)
        self.live_view_std.setGeometry(QtCore.QRect(190, 360, 71, 21))
        self.live_view_std.setObjectName("live_view_std")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "View Page"))
        self.lineEdit.setText(_translate("MainWindow", "Results"))
        self.lineEdit_2.setText(_translate("MainWindow", "Cam View"))
        self.lineEdit_5.setText(_translate("MainWindow", "X-Position"))
        self.lineEdit_6.setText(_translate("MainWindow", "Y-Position"))
        self.lineEdit_7.setText(_translate("MainWindow", "Std Long axis"))
        self.lineEdit_8.setText(_translate("MainWindow", "Std Short axis"))
        self.Circularity.setText(_translate("MainWindow", "Circularity"))
        self.X_Pos_Plot.setText(_translate("MainWindow", "Plot X-Position  Vs Frame"))
        self.Y_Pos_Plot.setText(_translate("MainWindow", "Plot Y-Position  Vs Frame"))
        self.FWHM_Plot.setText(_translate("MainWindow", "Plot FWHM"))
        self.Std_LA_Plot.setText(_translate("MainWindow", "Plot Std for Long Axis"))
        self.label.setText(_translate("MainWindow", "mm"))
        self.label_2.setText(_translate("MainWindow", "mm"))
        self.live_view_x.setText(_translate("MainWindow", "Live View"))
        self.live_view_y.setText(_translate("MainWindow", "Live View"))
        self.live_view_fwhm.setText(_translate("MainWindow", "Live View"))
        self.live_view_std.setText(_translate("MainWindow", "Live View"))
