#/bin/bash

source /home/pi/pyve/opencv/bin/activate

cd /home/pi/src
if [ ! -f opencv-3.3.0 ] ; then
    unzip opencv.zip
fi;

if [ ! -f opencv_contrib-3.3.0 ] ; then
    unzip opencv_contrib.zip
fi;


cd opencv-3.3.0
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/src/opencv_contrib-3.3.0/modules \
    -D ENABLE_NEON=ON \
    -D ENABLE_VFPV3=ON \
    -D BUILD_TESTS=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF ..
