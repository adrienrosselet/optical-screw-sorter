# optical-screw-sorter
This is the code for the optical screw sorting machine by @adrienrosselet

# Building dependencies
The CPP code (as of April 2019) requires OpenCV 3.1.0 or newer to be installed on target system with development dependencies.
Since Debian still includes OpenCV 2.4 as mainstream, one will need to compile newer version.
For Raspberry Pi users it's possible to follow the instructions here: https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/
Don't forget to build _**opencv-contrib**_ as well, since the code requires **xfeatures-2d** module.
The following command was used to configure OpenCV 3.4.6 during development:
`cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.6/modules -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF ..`

# Building
CMake should be used to build the code.

Just type the following commands in a sequence:
`cmake .`
`make`
This will result in a single dynamically linked executable file.

# Running
