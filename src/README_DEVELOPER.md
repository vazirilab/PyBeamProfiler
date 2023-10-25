# Developer Guide
This file contains some description of the classes and some of the functions in the main file (pybeamprofiler\src\pybeamprofiler_main.py), and a description of the files in the src file.

## Classes
Classes are descriped from line 1 to the end of code
1. `class Thread_FLIR(QtCore.QThread):` 
  That is the thread that controls the FLIR camera. It uses many functions from the package `PySpin`, which is the camera driver provided by the [producer](https://www.flir.com/products/spinnaker-sdk/?vertical=machine+vision&segment=iis). It emits a signal to the main class when a new frame is acquired. It returns the frame, the time stamp f the frame from the camera and the number of the frame.
1. `class Thread_Data(QtCore.QThread):` 
  This thread receives names of image files from the main thread and reads them one by one. It emits a signal to the main class when a new frame is read. It returns the frame, the time stamp of the frame from the camera and the number of the frame.
1. `class Thread_motorX(QtCore.QThread):` 
  This thread is used when the Position Control is in use(refer to the readme file). It receives the number of steps and direction the motor should move, and it sends them to the Arduino through the serial port.
1. `class Thread_FLIR_Ard(QtCore.QThread):`
  This thread is used when the Position Control is in use. It works similar to `class Thread_FLIR(QtCore.QThread):` except that it returns the position of the center of mass of the image.
1. `class PyBeamProfilerGUI(QtCore.QThread):`
  This is the main thread. It controls the inputs and outputs, saves results, Interact with the user, open tabs, etc. Most of the variables, initial values, signals and slots are introduced under `def __init__(self):`. All the functions used are within this class.

## Functions
Not all of the functions are mentioned here, just the unclear ones.
- `def closed_loop_start(self):` This function sets the starting point of the integral part of the PID controller in the closed loop.
- `def Follow_Position(self):` This function is called everytime the `class Thread_FLIR_Ard(QtCore.QThread):` thread emits a frame. It has the control loop in it.
- `def StartAcquisition_ard(self):` This function runs the `class Thread_FLIR_Ard(QtCore.QThread):` thread which starts the camera. It also Initialize the variables and changes them in case the user changed them.
- `def go_up(self):` sends 10 steps in the up direction (depends on the camera) to the `class Thread_motorY(QtCore.QThread):` thread.
- `def Caliberate_Motors(self):` This function connects the GUI to the Arduino board, add it to the threads that control the motors, and sets the up, down, left, and right direction.







