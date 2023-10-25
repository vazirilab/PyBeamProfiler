# PyBeamProfiler
A laser beam profiler app for FLIR cameras to monitor laser beam and control position using stepper motors. The GUI is very easy to use and manipulate and develop. 


## Installation
1. Clone this repository
1. Install any version of the Anaconda Python distribution. We highly recommend [Miniforge](https://github.com/conda-forge/miniforge), which is minimal and fast.
1. Create a new virtual environment using the `mamba` or `conda` command that comes with Anaconda/Miniforge and the supplied `beam_profile.yml` file (top level of the repository): `mamba env create -f beam_profile.yml`
1. Activate the environment: `mamba activate beam_profile`
1. Install the package used to control the FLIR camera `PySpin` by downloading the [Spinnaker SDK library](https://www.flir.com/products/spinnaker-sdk/?vertical=machine+vision&segment=iis).
1. Run the app: `python pybeamprofiler_main.py`

## Features and Usage
- You can either use it to monitor a live stream from a FLIR camera or you can load existing data (images from previous recordings).
- The GUI gives the user control over many parameters like the number of frames or the frame rate. It also allows the user to either set a specific exposure time or change it to be automatically set.
- If the user wants to save the results of a recording, they will have to select the number of frames contained in each .csv file. The first .csv file will be named (user selected name)_0.csv and the number goes up for the next file. The .csv contains the following values: 
  1. the time a frame is acquired. 
  2. The X-position of the center. 
  3. The Y-position of the center. 
  4. The std of the Gaussian distribution of the beam. 
  5. The FWHM of the Gaussian distribution of the beam.
- The profile of the beam can be saved as well by enabling the corresponding checkbox
- If the Saving option is selected while recording from "Position Control", the saved .csv file will contain: 
  1. the timestamp of an  acquired frame.
  2. The X-position of the center. 
  3. The Y-position of the center. 
  4. Loop response in X. 
  5. Loop response in Y. 
  6. Time of frame according to PC.
- In case The user wants to add new featurs:
  1. Use [qt designer](https://doc.qt.io/qt-6/qtdesigner-manual.html) to change the Tabs files (.ui files) or add one.
  2. Change the .ui files to .py files by typing ` pyuic5 -x main.ui -o main.py` in the conda command window.
  3. Add the function or the thread to the main code and connect it to the new element you added. If you designed a new app import the class in the .py you created.
- The User can use the "Position Control" tap to control the laser with an arduino and a stepper motor. Design is in the "Hardware_design_control" folder, and the code for the arduino can be found at "src/Ard_control/Arduino_stepper_control".
- Note: Due to the camera internal memory the time stamps of the acquired frames are wrapped. Go to "pybeamprofiler\test\Unwrap_time_data.m" which is a MATLAB file that unwraps the data to time in seconds.

## Position Stabilizing Design
- In case the user wishes to use the position stabilizing design offered in the GUI, please do as follows:
  1. Go to the "pybeamprofiler\Hardware_design_control" folder and order the parts specified in "Parts_List.xlsx".
  1. Open the "Motor_mirror_mount.iam" assembly to view how the parts are Installed together and follow that scheme.
  1. The part "Stage.ipt" should be 3D printed or machined.
  1. To program the Arduino go to "pybeamprofiler\src\Ard_control\Arduino_stepper_control" and use [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE) to upload the Arduino code in "Arduino_stepper_control.ino" to the Arduino
  1. Connect the stepper motor to the driver as stated in the manual of both products, and connect the pins of the driver (e.g. PUL, DIR etc) to the digital pins on the Arduino. Refer to "Arduino_stepper_control.ino" to know the correct configration of the digetal pins (e.g. PUL pin on the X-driver to PIN 8 on the Arduino etc).
  1. Connect each of the drivers to a 12V power supply. and use the switches to control the steps/revolution and current to the motors (check the manual first).
  1. Connect the Arduino to the PC and open the Position Control tab in the GUI. 
  1. You have the option to tune the PID controller and move the motors.
- The design was tested and showed reduction of slow drift and low frequency (> 0.1 Hz) movement, and, when tuned properly, can reduce high frequency movement by 5 dB. Refer to "pybeamprofiler\data\PyBeamProfiler_Poster_Shehata.pdf" for more information.

## License
Copyright (c) 2023 Mohamed Shehata

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, and/or sublicense copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
- The user agrees to acknowledge the source of the SOFTWARE in any publications reporting use of it.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Main Developer
- Mohamed Shehata, Carl von Ossietzky Universität Oldenburg

## Acknowledgments
Special thanks to the following people for their great support and help with developing the software
- Tobias Önol-Nöbauer, Ph.D., The Rockefeller University 
- Alipasha Vaziri, Ph.D., The Rockefeller University
- Philipp Huke, Prof. Dr.-Ing., Hochschule Emden/Leer
- Lars Jepsen, M. Sc., Hochschule Emden/Leer
- Gunnar Scharf, Carl von Ossietzky Universität Oldenburg



