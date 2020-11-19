# OpenHSML
OpenHSML - Open-source Hybrid Stereovision Matching Library.

##  Description
We propose a simple approach for finding the point correspondence between an RGB and a depth image. This approach uses the principles of stereovision to find the points of interest in the depth image without going through image alignment. Our method requires a quick and simple calibration, without the need for a checkerboard, to determine the stereo model. This model allows us to solve our problem without having to determine the camera parameters.

## Project organization
The library OpenHSML is written in C++ and developed on the Unix system Ubuntu 18.04.

The OpenHSML project hierarchy is the following:
- apps: examples to help get started with OpenHSML.
- bin: executables output directory.
- build: build directory.
- share: files/folders necessary for calibration.
- src: source files.
- include: header files, with the same structure as src.

A set of images is provided along with the library, in the following folders: *path/to/library/share/resources/calibration/2d* for the RGB images and *path/to/library/share/resources/calibration/depth* for the depth images.


##  Installation
### Clone the project:
First clone the project and go in the folder cloned.
```
git clone https://github.com/jordan-nowak/OpenHSML.git
cd OpenHSML/
```

<!--
### Automatic installation:
Then run the install script on your linux system to install all dependencies (OpenCV, Eigen and yaml-cpp):
```
...
```
Remark: for OpenCV all 2.X and 3.X versions are compatible.
-->

### Manual installation:
#### Install CMake GUI:
- Start by updating the packages list:
```
sudo apt -y update
sudo apt -y upgrade
```

- Install the build-essential and cmake package by typing:
```
sudo apt-get install build-essential
sudo apt-get install cmake
```

- To validate that the GCC compiler is successfully installed, use the `gcc --version` command which prints the GCC version.

#### Install Doxygen to generate the software documentation:
```
sudo apt install graphviz doxygen doxygen-gui
```

<!-- Pour le moment -->
#### Install Eigen library (local):
```
cd path/to/OpenHSML/include/source_lib/
git clone https://gitlab.com/libeigen/eigen.git
cd eigen/
cp -r Eigen ../../lib/
```

#### Install OpenCV library (local):
```
cd path/to/OpenHSML/include/source_lib/
cwd=$(pwd) && cvVersion="3.4.12"
mkdir opencv_src
cd opencv_src
git clone https://github.com/opencv/opencv.git
cd opencv/ && git checkout $cvVersion
mkdir build && cd build
```
```
cmake -D GLIBCXX_USE_CXX11_ABI=0 \
      -D CMAKE_BUILD_TYPE=RELEASE \
      -D ENABLE_FAST_MATH=ON \
      -D CMAKE_INSTALL_PREFIX=$cwd/opencv_src/opencv_lib \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_CUDA=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF ..
```
```
make -j`nproc` && make install
```
```
cd ../../ && cp -r opencv_lib/ ../../lib/
cd ../../lib/ && mv opencv_lib/ opencv/
```

#### Install yaml-cpp library:
```
cd path/to/OpenHSML/include/source_lib/
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp/ && mkdir build && cd build
cmake ..
make
cd ../include/
cp -r yaml-cpp/ ../../../lib/
```

#### Install matplotlib-cpp library (local):
```
sudo apt-get install python-matplotlib python-numpy python2.7-dev
cd path/to/OpenHSML/include/lib/
git clone https://github.com/lava/matplotlib-cpp.git
```

##  Getting Started
###  Build the project
To build the library, in a terminal do the following command:
```
cd path/to/OpenHSML/build
cmake ..
make
```

### Demonstration
If you have not yet modified the *share* folder, a small demonstration is available.

#### Application
To launch the demo apps, do the following command:
```
cd path/to/OpenHSML/
./bin/demo -test -display
```

The `-test` argument allows to launch the demo mode which will take the images (RGB and depth) pre-recorded in the *path/to/OpenHSML/share/resources/img/* folder.

The `-display` argument allows us to display the different steps and the results obtained for the selected points.

Indeed, when launching this command, you will be asked to click on the RGB image presented on your screen to select points. When you are done selecting the points, a simple press on the `ESC` key will launch the estimation.

<!-- Numeric tools -->

#### Calibration tutorial
If you want to test the calibration with our demo apps, you can launch with the following command:
```
./bin/demo -calibration
```

Remark: If you put your own images in the calibration folder *path/to/OpenHSML/share/resources/calibration/*, make sure you save them in the right format. The RGB images can be saved in any image format readable by OpenCV (for example *.png* or *.jpg*). The depth images are save in a YAML file with [storage class](https://docs.opencv.org/3.4.12/da/d56/classcv_1_1FileStorage.html) in OpenCV.

##### Parameters necessary
First, the program asks the user to modified the default values of the parameters (HFOV, VFOV, cu, cv, width and height) if necessary.

If you only provide a depth image on the z-axis, the parameters HFOV, VFOV, cu and cv are required. Indeed, in this case, these parameters permit to estimate the position of the 3D points on the x and y-axis. However, it is possible not to make this step if you directly give the depth images with the 3D point coordinates following axes $x$, $y$, and $z$. In any case, the size of the depth image must be known and given in this first part of the calibration. This will allow the algorithm to properly read the file containing the depth image.

##### Set of points
Then, the program ask you if you want to calibrate the model by doing point matching in the scrolling images. In this step, the algorithm displays the two images side by side. The image on the left will be the image with which the user can always interact to select points. Therefore, when selecting a point in the left image, the images swap places to allow selection of the point in the second image.

To facilitate calibration, it is possible to activate several selection modes, via the keyboard:
- the `p` key activates the *point* mode, which is the default mode, and allows stitch by stitch selection;
- the `l` key activates the *line* mode, allowing a certain number of points to be sampled homogeneously between two selected points;
- the `q` key activates the *quadrilater* mode, allowing a multitude of points to be sampled homogeneously within four selected points.

For these digital tools to work properly, you must select the points in both images in the same order. To move to the next image, you should press the `ESC` key.


##  About authors
OpenHSML has been developped by following authors: Jordan Nowak (LIRMM)

Please contact Jordan Nowak (nowak@lirmm.fr) - LIRMM for more information or questions.

<!-- ##  Reference and documentation -->
