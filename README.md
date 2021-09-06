
openhsml
==============

A simple library for finding the point correspondence between an RGB and a depth image.

# Table of Contents
 - [Package Overview](#package-overview)
 - [Installation and Usage](#installation-and-usage)
 - [Offline API Documentation](#offline-api-documentation)
 - [License](#license)
 - [Authors](#authors)




Package Overview
================

The **openhsml** package contains the following:

 * Libraries:

   * OpenHSML-shared (shared)

 * Applications:

   * OpenHSML_apps


Installation and Usage
======================

The **openhsml** project is packaged using [PID](http://pid.lirmm.net), a build and deployment system based on CMake.

If you wish to adopt PID for your develoment please first follow the installation procedure [here](http://pid.lirmm.net/pid-framework/pages/install.html).

If you already are a PID user or wish to integrate **openhsml** in your current build system, please read the appropriate section below.


## Using an existing PID workspace

This method is for developers who want to install and access **openhsml** from their PID workspace.

You can use the `deploy` command to manually install **openhsml** in the workspace:
```
cd <path to pid workspace>
pid deploy package=openhsml # latest version
# OR
pid deploy package=openhsml version=x.y.z # specific version
```
Alternatively you can simply declare a dependency to **openhsml** in your package's `CMakeLists.txt` and let PID handle everything:
```
PID_Dependency(openhsml) # any version
# OR
PID_Dependency(openhsml VERSION x.y.z) # any version compatible with x.y.z
```

If you need more control over your dependency declaration, please look at [PID_Dependency](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-dependency) documentation.

Once the package dependency has been added, you can use `openhsml/OpenHSML-shared` as a component dependency.

You can read [PID_Component](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-component) and [PID_Component_Dependency](https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html#pid-component-dependency) documentations for more details.
## Standalone installation

This method allows to build the package without having to create a PID workspace manually. This method is UNIX only.

All you need to do is to first clone the package locally and then run the installation script:
 ```
git clone git@gite.lirmm.fr:nowak/openhsml.git
cd openhsml
./share/install/standalone_install.sh
```
The package as well as its dependencies will be deployed under `binaries/pid-workspace`.

You can pass `--help` to the script to list the available options.

### Using **openhsml** in a CMake project
There are two ways to integrate **openhsml** in CMake project: the external API or a system install.

The first one doesn't require the installation of files outside of the package itself and so is well suited when used as a Git submodule for example.
Please read [this page](https://pid.lirmm.net/pid-framework/pages/external_API_tutorial.html#using-cmake) for more information.

The second option is more traditional as it installs the package and its dependencies in a given system folder which can then be retrived using `find_package(openhsml)`.
You can pass the `--install <path>` option to the installation script to perform the installation and then follow [these steps](https://pid.lirmm.net/pid-framework/pages/external_API_tutorial.html#third-step--extra-system-configuration-required) to configure your environment, find PID packages and link with their components.
### Using **openhsml** with pkg-config
You can pass `--pkg-config on` to the installation script to generate the necessary pkg-config files.
Upon completion, the script will tell you how to set the `PKG_CONFIG_PATH` environment variable for **openhsml** to be discoverable.

Then, to get the necessary compilation flags run:

```
pkg-config --static --cflags openhsml_OpenHSML-shared
```

```
pkg-config --variable=c_standard openhsml_OpenHSML-shared
```

```
pkg-config --variable=cxx_standard openhsml_OpenHSML-shared
```

To get linker flags run:

```
pkg-config --static --libs openhsml_OpenHSML-shared
```




Offline API Documentation
=========================

With [Doxygen](https://www.doxygen.nl) installed, the API documentation can be built locally by turning the `BUILD_API_DOC` CMake option `ON` and running the `doc` target, e.g
```
pid cd openhsml
pid -DBUILD_API_DOC=ON doc
```
The resulting documentation can be accessed by opening `<path to openhsml>/build/release/share/doc/html/index.html` in a web browser.

License
=======

The license that applies to the whole package content is **GNULGPL**. Please look at the [license.txt](./license.txt) file at the root of this repository for more details.

Authors
=======

**openhsml** has been developed by the following authors: 
+ Nowak Jordan ()

Please contact Nowak Jordan -  for more information or questions.
