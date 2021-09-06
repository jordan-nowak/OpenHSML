OpenHSML - Open-source Hybrid Stereovision Matching Library.
==============
##  Description
We propose a simple approach for finding the point correspondence between an RGB and a depth image. This approach uses the principles of stereovision to find the points of interest in the depth image without going through image alignment. Our method requires a quick and simple calibration, without the need for a checkerboard, to determine the stereo model. This model allows us to solve our problem without having to determine the camera parameters.

# Table of Contents
 - [Package Overview](#package-overview)
 - [Installation and Usage](#installation-and-usage)
 - [Offline API Documentation](#offline-api-documentation)
 - [License](#license)
 - [Authors](#authors)




Package Overview
================

The library OpenHSML is written in C++ and developed on the Unix system Ubuntu 18.04.

The OpenHSML project hierarchy is the following:
- apps: an example to help get started with OpenHSML : **OpenHSML_apps**.
- build: build directory.
- share: files/folders necessary for calibration.
- src: source files.
- include: header files, with the same structure as src.

A set of images is provided along with the library, in the following folders: *path/to/library/share/resources/calibration/2d* for the RGB images and *path/to/library/share/resources/calibration/depth* for the depth images.

Installation and Usage
======================

The **OpenHSML** project is packaged using [PID](http://pid.lirmm.net), a build and deployment system based on CMake.

If you wish to adopt PID for your develoment please first follow the installation procedure [here](http://pid.lirmm.net/pid-framework/pages/install.html).

If you already are a PID user or wish to integrate **OpenHSML** in your current build system, please read the appropriate section below.


## Using an existing PID workspace

First clone the project and go in the folder cloned:
```
cd path/to/pid-workspace/packages/
git clone https://github.com/jordan-nowak/OpenHSML.git
cd OpenHSML/
```

Then, go on the integration branch:
```
git checkout integration
```

To finish, you start the code compilation as follows:
```
pid build
```

Once the package dependency has been added, you can use `OpenHSML/OpenHSML-shared` as a component dependency.

You can read [PID_Component](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-component) and [PID_Component_Dependency](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-component-dependency) documentations for more details.

## Standalone installation

This method allows to build the package without having to create a PID workspace manually. This method is UNIX only.

All you need to do is to first clone the package locally and then run the installation script:
 ```
git clone git@gite.lirmm.fr:nowak/OpenHSML.git
cd OpenHSML
git checkout integration
./share/install/standalone_install.sh
```
The package as well as its dependencies will be deployed under `binaries/pid-workspace`.

You can pass `--help` to the script to list the available options.

### Using **OpenHSML** in a CMake project
There are two ways to integrate **OpenHSML** in CMake project: the external API or a system install.

The first one doesn't require the installation of files outside of the package itself and so is well suited when used as a Git submodule for example.
Please read [this page](https://pid.lirmm.net/pid-framework/pages/external_API_tutorial.html#using-cmake) for more information.

The second option is more traditional as it installs the package and its dependencies in a given system folder which can then be retrived using `find_package(OpenHSML)`.
You can pass the `--install <path>` option to the installation script to perform the installation and then follow [these steps](https://pid.lirmm.net/pid-framework/pages/external_API_tutorial.html#third-step--extra-system-configuration-required) to configure your environment, find PID packages and link with their components.

### Using **OpenHSML** with pkg-config
You can pass `--pkg-config on` to the installation script to generate the necessary pkg-config files.
Upon completion, the script will tell you how to set the `PKG_CONFIG_PATH` environment variable for **OpenHSML** to be discoverable.

Then, to get the necessary compilation flags run:

```
pkg-config --static --cflags OpenHSML_OpenHSML-shared
```

```
pkg-config --variable=c_standard OpenHSML_OpenHSML-shared
```

```
pkg-config --variable=cxx_standard OpenHSML_OpenHSML-shared
```

To get linker flags run:

```
pkg-config --static --libs OpenHSML_OpenHSML-shared
```


##  Usage - Getting Started
###  Build the project
To build the library, in a terminal do the following command:
```
cd path/to/pid-workspace/packages/OpenHSML/
pid build
```

### Demonstration
If you have not yet modified the *share* folder, a small demonstration is available.

#### Application
To launch the demo apps, do the following command:
```
cd path/to/pid-workspace/
./install/x86_64_linux_stdc++11/OpenHSML/0.1.0/bin/OpenHSML_OpenHSML_apps -test -display
```

The `-test` argument allows to launch the demo mode which will take the images (RGB and depth) pre-recorded in the *path/to/OpenHSML/share/resources/img/* folder.

The `-display` argument allows us to display the different steps and the results obtained for the selected points.

Indeed, when launching this command, you will be asked to click on the RGB image presented on your screen to select points. When you are done selecting the points, a simple press on the `ESC` key will launch the estimation.

#### Calibration tutorial
If you want to test the calibration with our demo apps, you can launch with the following command:
```
./install/x86_64_linux_stdc++11/OpenHSML/0.1.0/bin/OpenHSML_OpenHSML_apps -calibration
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


Offline API Documentation
=========================

With [Doxygen](https://www.doxygen.nl) installed, the API documentation can be built locally by turning the `BUILD_API_DOC` CMake option `ON` and running the `doc` target, e.g
```
pid cd OpenHSML
pid -DBUILD_API_DOC=ON doc
```
The resulting documentation can be accessed by opening `<path to OpenHSML>/build/release/share/doc/html/index.html` in a web browser.

License
=======

The license that applies to the whole package content is **GNU Lesser General Public License version 3 (GNU GPLv3)**. Please look at the [license.txt](./license.txt) file at the root of this repository for more details.

Authors
=======

OpenHSML has been developped by following authors:
+ Nowak Jordan (LIRMM)

Please contact Nowak Jordan (nowak@lirmm.fr) - LIRMM for more information or questions.