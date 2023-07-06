# pybeamprofiler
A laser beam profiler app for FLIR cameras. You can either use it to monitor a live stream from a FLIR camera or you can load existing data (images from previous recordings). The GUI gives the user control over many parameters like the number of frames or the frame rate. It also allows the user to either set a specific exposure time or change it to be automatically set. If the user wants to save the results of a recording, they will have to select the number of frames contained in each .csv file. The first .csv file will be named (user selected name)_0.csv and the number goes up for the next file. The .csv contains the following values: 1- the time a frame is acquired. 2- The X-position of the center. 3- The Y-position of the center. 4- The std of the Gaussian distribution of the beam. 5- The FWHM of the Gaussian distribution of the beam.

##########INSTALLATION#########
To run the code you simply need to run the Python file in src named: pybeamprofiler_main.py
to run this file, the environment in which you run it needs to have these packages installed:
PyQt5
skimage
numpy 
matplotlib
glob
os
PIL
math
time
scipy
PySpin
cv2
pyqtgraph
pip

Regarding the package PySpin, it is a library offered by the company producing the FLIR cam. To install it go to this link (https://www.flir.com/products/spinnaker-sdk/?vertical=machine%20vision&segment=iis) or use the Dropbox link(https://www.dropbox.com/sh/lypn6q22xdjlkf2/AAB45QydUTdl-r48MmsRhnVja?dl=0) and download this zip file (spinnaker_python-3.1.0.79-cp310-cp310-win_amd64.zip) then follow the instruction in the README.txt file.

