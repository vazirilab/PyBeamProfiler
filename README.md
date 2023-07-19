# pybeamprofiler
A laser beam profiler app for FLIR cameras.

## Features and Usage
- You can either use it to monitor a live stream from a FLIR camera or you can load existing data (images from previous recordings).
- The GUI gives the user control over many parameters like the number of frames or the frame rate. It also allows the user to either set a specific exposure time or change it to be automatically set.
- - If the user wants to save the results of a recording, they will have to select the number of frames contained in each .csv file. The first .csv file will be named (user selected name)_0.csv and the number goes up for the next file. The .csv contains the following values: 1- the time a frame is acquired. 2- The X-position of the center. 3- The Y-position of the center. 4- The std of the Gaussian distribution of the beam. 5- The FWHM of the Gaussian distribution of the beam.

## Installation
1. Clone this repository
1. Install any version of the Anaconda Python distribution. We highly recommend [Miniforge](https://github.com/conda-forge/miniforge), which is minimal and fast.
1. Create a new virtual environment using the `mamba` or `conda` command that comes with Anaconda/Miniforge and the supplied `beam_profile.yml` file (top level of the repository): `mamba env create -f beam_profile.yml`
1. Activate the environment: `mamba activate beam_profile`
1. Go to the directory `<Path>\pybeamprofiler-main\src\Spinker_files` to install `PySpin` (the package used to control the FLIR camera) using pip: `pip install spinnaker_python-3.1.0.79-cp310-cp310-win_amd64.whl`
1. Tun the app: `python pybeamprofiler_main.py`

