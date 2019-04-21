# optical-screw-sorter
This is the code for the optical screw sorting machine by @adrienrosselet

# Building dependencies
The CPP code (as of April 2019) requires OpenCV 3.1.0 or newer to be installed on target system with development dependencies.
Since Debian still includes OpenCV 2.4 as mainstream, one will need to compile newer version.
For Raspberry Pi users it's possible to follow the instructions here: https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/
Don't forget to build _**opencv-contrib**_ as well, since the code requires **xfeatures-2d** module.

# Building
CMake should be used to build the code.

Just type the following commands in a sequence:
`cmake .`
`make`
This will result in a single dynamically linked executable file.

# Running
