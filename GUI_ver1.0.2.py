# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'd:\PythonProject\single UAV coverage\ui\test.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


'''
Ver 1.0.3.20220815_alpha
'''


from logging import warning
from lzma import is_check_supported
from tkinter.tix import WINDOW
#from typing_extensions import Self
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSlot
import sys
import re
import os
from sqlalchemy import Unicode
import MultiAgent
#import class_def

from PyQt5.QtGui import QIntValidator,QDoubleValidator,QRegExpValidator
from PyQt5.QtCore import QRegExp
import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *




class Ui_MainWindow(object):

    openpath=''
    savepath=''
    Coordinate=[]
    UAVPerformance=[]
    Radius=[]
    Alpha=[]
    StartPoint=[]
    Velocity=[]

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(786, 540)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setFixedSize(MainWindow.width(),MainWindow.height())
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(40, 50, 700, 200))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.groupBox.setFont(font)
        self.groupBox.setStyleSheet("QGroupBox::title{\n"
            "font-size: 13px;\n"
            "font-weight: bold;\n"
            "}")
        self.groupBox.setObjectName("groupBox")
        self.pushButton_2 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_2.setGeometry(QtCore.QRect(70, 90, 120, 30))
        self.pushButton_2.setObjectName("pushButton")

        self.pushButton = QtWidgets.QPushButton(self.groupBox)
        self.pushButton.setGeometry(QtCore.QRect(70, 50, 120, 30))
        self.pushButton.setObjectName("pushButton_2")

        self.lineEdit = QtWidgets.QLineEdit(self.groupBox)
        self.lineEdit.setGeometry(QtCore.QRect(210, 90, 400, 30))
        self.lineEdit.setObjectName("lineEdit")
        self.pushButton_3 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_3.setGeometry(QtCore.QRect(620, 90, 41, 30))
        self.pushButton_3.setObjectName("pushButton_3")
        self.label_2 = QtWidgets.QLabel(self.groupBox)
        self.label_2.setGeometry(QtCore.QRect(210, 140, 451, 31))
        font = QtGui.QFont()
        font.setFamily("Adobe Devanagari")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(40, 270, 700, 200))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_2.setFont(font)
        self.groupBox_2.setStyleSheet("QGroupBox::title{\n"
            "font-size: 13px;\n"
            "font-weight: bold;\n"
            "}")
        self.groupBox_2.setObjectName("groupBox_2")
        self.groupBox_3 = QtWidgets.QGroupBox(self.groupBox_2)
        self.groupBox_3.setGeometry(QtCore.QRect(30, 30, 181, 151))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_3.setFont(font)
        self.groupBox_3.setObjectName("groupBox_3")
        self.checkBox_2 = QtWidgets.QCheckBox(self.groupBox_3)
        self.checkBox_2.setGeometry(QtCore.QRect(20, 30, 151, 19))
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox_3 = QtWidgets.QCheckBox(self.groupBox_3)
        self.checkBox_3.setGeometry(QtCore.QRect(20, 70, 91, 19))
        self.checkBox_3.setObjectName("checkBox_3")
        self.checkBox_4 = QtWidgets.QCheckBox(self.groupBox_3)
        self.checkBox_4.setGeometry(QtCore.QRect(20, 110, 91, 19))
        self.checkBox_4.setObjectName("checkBox_4")
        self.groupBox_4 = QtWidgets.QGroupBox(self.groupBox_2)
        self.groupBox_4.setGeometry(QtCore.QRect(240, 30, 441, 151))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.groupBox_4.setFont(font)
        self.groupBox_4.setObjectName("groupBox_4")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_2.setGeometry(QtCore.QRect(100, 110, 261, 30))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.pushButton_5 = QtWidgets.QPushButton(self.groupBox_4)
        self.pushButton_5.setGeometry(QtCore.QRect(380, 110, 41, 30))
        self.pushButton_5.setObjectName("pushButton_5")
        self.checkBox_5 = QtWidgets.QCheckBox(self.groupBox_4)
        self.checkBox_5.setGeometry(QtCore.QRect(20, 30, 241, 19))
        self.checkBox_5.setObjectName("checkBox_5")
        self.checkBox_6 = QtWidgets.QCheckBox(self.groupBox_4)
        self.checkBox_6.setGeometry(QtCore.QRect(20, 70, 231, 19))
        self.checkBox_6.setObjectName("checkBox_6")
        self.label_4 = QtWidgets.QLabel(self.groupBox_4)
        self.label_4.setGeometry(QtCore.QRect(20, 110, 71, 31))
        font = QtGui.QFont()
        font.setFamily("Adobe Devanagari")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_6.setGeometry(QtCore.QRect(170, 480, 93, 28))
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_7 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_7.setGeometry(QtCore.QRect(500, 480, 93, 28))
        self.pushButton_7.setObjectName("pushButton_7")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(150, 10, 501, 31))
        font = QtGui.QFont()
        font.setFamily("Adobe Devanagari")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(680, 500, 100, 16))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setBold(False)
        font.setWeight(50)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 786, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "面向对地搜索的多无人机协同区域覆盖航迹路线生成软件V1.0"))
        self.groupBox.setTitle(_translate("MainWindow", "任务数据读入"))

        self.pushButton.setText(_translate("MainWindow", "一键读入"))
        self.pushButton.clicked.connect(self.ReadFromDefault)

        self.pushButton_2.setText(_translate("MainWindow", "从路径读入"))
        self.pushButton_2.clicked.connect(self.ReadFromBrowse)

        self.pushButton_3.setText(_translate("MainWindow", "浏览"))
        self.pushButton_3.clicked.connect(self.OpenBrowse)

        self.groupBox_2.setTitle(_translate("MainWindow", "仿真首选项"))
        self.groupBox_3.setTitle(_translate("MainWindow", "算法代价"))
        self.checkBox_2.setText(_translate("MainWindow", "总路程(不含往返)"))
        self.checkBox_3.setText(_translate("MainWindow", "转弯数"))
        self.checkBox_4.setText(_translate("MainWindow", "任务时间"))
        self.groupBox_4.setTitle(_translate("MainWindow", "输出参数"))

        self.pushButton_5.setText(_translate("MainWindow", "浏览"))
        self.pushButton_5.clicked.connect(self.SaveBrowse)

        self.checkBox_5.setText(_translate("MainWindow", "保存路径点(含代价)文件(.txt)"))
        self.checkBox_6.setText(_translate("MainWindow", "保存路径图(.png)"))

        self.pushButton_6.setText(_translate("MainWindow", "仿真计算"))
        self.pushButton_6.clicked.connect(self.calc)

        self.pushButton_7.setText(_translate("MainWindow", "退出"))
        self.pushButton_7.clicked.connect(QtWidgets.QWidget.close)



        self.label.setText(_translate("MainWindow", "面向对地搜索的多无人机协同区域覆盖航迹路线生成软件V1.0"))
        #self.title.setText(_translate("MainWindow", "面向对地搜索的多无人机协同区域覆盖航迹路线生成系统 V1.0"))
        self.label_2.setText(_translate("MainWindow", "Waiting for data input..."))
        self.label_3.setText(_translate("MainWindow", "V1.0.3.20220815"))
        self.label_4.setText(_translate("MainWindow", "保存路径"))


    def ReadFromDefault(self):
        try:
            f = open('input.txt','r',encoding='UTF-8')
            InputData = f.readlines()
            self.ReProcess(InputData)
            QtWidgets.QMessageBox.information(QtWidgets.QWidget(),'Succeeded','Data Imported!',\
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
        except:
            QtWidgets.QMessageBox.warning(QtWidgets.QWidget(),'Warning','Invalid Input!',\
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
        finally:
            f.close()

    def ReadFromBrowse(self):
        if self.lineEdit.text() == '':
            QtWidgets.QMessageBox.warning(QtWidgets.QWidget(),'Warning','Invalid Input!',\
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
        else:
            try:
                f = open(self.lineEdit.text(),'r',encoding='UTF-8')
                InputData = f.readlines()
                self.ReProcess(InputData)
                QtWidgets.QMessageBox.information(QtWidgets.QWidget(),'Succeeded','Data Imported!',\
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
            except:
                QtWidgets.QMessageBox.warning(QtWidgets.QWidget(),'Warning','Unknown Error',\
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
            finally:
                f.close()

    def ReProcess(self,data):
        for i in range(len(data)):
            data[i] = data[i].rstrip('\n')
            data[i] = data[i].replace(' ','')
        flags = []

        #re for processing input mission area
        coord = re.findall(r"(-?\d+\.?\d*)\D(-?\d+\.?\d*)",data[0])
        self.Coordinate = []
        for i in range(len(coord)):
            coordtemp = (float(coord[i][0]),float(coord[i][1]))
            self.Coordinate.append(coordtemp)
        print(self.Coordinate)

        #re for processing input UAV performance
        performance = re.findall(r"-?\d+\.?\d*",data[1])
        self.UAVPerformance = []
        for i in range(len(performance)):
            self.UAVPerformance.append(float(performance[i]))
        flags.append(len(self.UAVPerformance))
        print(self.UAVPerformance)
        
        #re for processing input radius
        radii = re.findall(r"-?\d+\.?\d*",data[2])
        self.Radius = []
        for i in range(len(radii)):
            self.Radius.append(float(radii[i]))
        flags.append(len(self.Radius))
        print(self.Radius)

        #re for processing input alpha value
        alp = re.findall(r"-?\d+\.?\d*",data[3])
        self.Alpha = []
        for i in range(len(alp)):
            self.Alpha.append(float(alp[i]))
        flags.append(len(self.Alpha))
        print(self.Alpha)

        #re for processing input start point coordinate
        startP = re.findall(r"-?\d+\.?\d*",data[4])
        self.StartPoint=[]
        for i in range(len(startP)):
            self.StartPoint.append(float(startP[i]))
        lensp = len(self.StartPoint)
        print(self.StartPoint)

        #re for processing UAV velocity
        v = re.findall(r"(-?\d+\.?\d*)\D(-?\d+\.?\d*)",data[5])
        self.Velocity = []
        for i in range(len(v)):
            vTemp = (float(v[i][0]),float(v[i][1]))
            self.Velocity.append(vTemp)
        flags.append(len(self.Velocity))
        print(self.Velocity)

        iscompatible = True
        if lensp != 2:
            QtWidgets.QMessageBox.warning(QtWidgets.QWidget(),'Warning','Invalid Input:\nStart point must be a 2-d coordinate!',\
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
        else:
            for i in range(len(flags)-1):
                for j in range(i+1,len(flags)):
                    if flags[i] != flags[j]:
                        iscompatible = False

        if iscompatible == False:        
            QtWidgets.QMessageBox.warning(QtWidgets.QWidget(),'Warning','Invalid Input:\nIncompatible Data!',\
            QtWidgets.QMessageBox.Yes |  QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)


    def OpenBrowse(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(QtWidgets.QDialog(),'Open File','d:\\','Text Files (*.txt)')
        self.lineEdit.setText(fname)
        self.openpath = fname
        self.label_2.setText('File Loaded.')

    def SaveBrowse(self):
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(QtWidgets.QDialog(),'Save File','c:\\','Text Files (*.txt)')
        self.lineEdit_2.setText(fname)
        self.savepath = fname

    def calc(self):

        #try:

            para = ''
            if self.checkBox_5.isChecked():
                para += '1'
            else:
                para += '0'
            if self.checkBox_6.isChecked():
                para += '1'
            else:
                para += '0'
            if self.checkBox_2.isChecked():
                para += '1'
            else:
                para += '0'
            if self.checkBox_3.isChecked():
                para += '1'
            else:
                para += '0'
            if self.checkBox_4.isChecked():
                para += '1'
            else:
                para += '0'

            if self.lineEdit_2.text() == '':
                SavePath1 = 'Output.txt'
                SavePath2 = 'Coverage Route.png'
            else:
                SavePath1 = self.lineEdit_2.text()
                SavePath2 = (self.lineEdit_2.text()).replace('.txt','.png')

            print(para)
            print(len(self.UAVPerformance))
            print(self.UAVPerformance)
            MultiAgent.plot_multi_cpp(self.Coordinate,len(self.UAVPerformance),self.UAVPerformance,\
            self.Radius,self.Alpha,self.Velocity,self.StartPoint,SavePath1,SavePath2,para)

            
            QtWidgets.QMessageBox.information(QtWidgets.QWidget(),'','Simulation Completed!',\
            QtWidgets.QMessageBox.Yes |  QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)

            
            if self.lineEdit_2.text() == '':
                UrlPath = os.path.abspath(os.curdir)
            else:
                
                UrlPath = (self.lineEdit_2.text()).replace(os.path.basename(self.lineEdit_2.text()),'')
            
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(UrlPath))
        #except:
            #QtWidgets.QMessageBox.warning(QtWidgets.QWidget(),'Warning','Incomplete Parameters!',\
            #QtWidgets.QMessageBox.Yes |  QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
    
        
        

    

if __name__ == "__main__":
    app=QtWidgets.QApplication(sys.argv)
    MainWindow=QtWidgets.QMainWindow()
    ui=Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

