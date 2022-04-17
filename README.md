
## Cloning the Repo

To download the code, navigate to a folder of your choosing on the Jetson.  First, make sure git and cmake are installed:

``` bash
$ sudo apt-get update
$ sudo apt-get install git cmake
```

Then clone the `jetson-inference` project:

``` bash
$ git clone https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ git submodule update --init
```

Remember to run the `git submodule update --init` step (or clone with the `--recursive` flag).

## Python Development Packages

The Python functionality of this project is implemented through Python extension modules that provide bindings to the native C++ code using the Python C API.  While configuring the project, the repo searches for versions of Python that have development packages installed on the system, and will then build the bindings for each version of Python that's present (e.g. Python 2.7, 3.6, and 3.7).  It will also build numpy bindings for versions of numpy that are installed.

By default, Ubuntu comes with the `libpython-dev` and `python-numpy` packages pre-installed (which are for Python 2.7).  Although the Python 3.6 interpreter is pre-installed by Ubuntu, the Python 3.6 development packages (`libpython3-dev`) and `python3-numpy` are not.  These development packages are required for the bindings to build using the Python C API.  

So if you want the project to create bindings for Python 3.6, install these packages before proceeding:

``` bash
$ sudo apt-get install libpython3-dev python3-numpy
``` 

Installing these additional packages will enable the repo to build the extension bindings for Python 3.6, in addition to Python 2.7 (which is already pre-installed).  Then after the build process, the [`jetson.inference`](https://rawgit.com/dusty-nv/jetson-inference/python/docs/html/python/jetson.inference.html) and [`jetson.utils`](https://rawgit.com/dusty-nv/jetson-inference/python/docs/html/python/jetson.utils.html) packages will be available to use within your Python environments.


## Configuring with CMake

Next, create a build directory within the project and run `cmake` to configure the build.  When `cmake` is run, a script is launched ([`CMakePreBuild.sh`](../CMakePreBuild.sh)) that will install any required dependencies and download DNN models for you.

``` bash
$ cd jetson-inference    # omit if working directory is already jetson-inference/ from above
$ mkdir build
$ cd build
$ cmake ../
```


## Downloading Models

The project comes with many pre-trained networks that can you can choose to have downloaded and installed through the **Model Downloader** tool ([`download-models.sh`](../tools/download-models.sh)).  By default, not all of the models are initially selected for download to save disk space.  You can select the models you want, or run the tool again later to download more models another time.

When initially configuring the project, `cmake` will automatically run the downloader tool for you:

<img src="https://raw.githubusercontent.com/dusty-nv/jetson-inference/python/docs/images/download-models.jpg" width="650">

To run the Model Downloader tool again later, you can use the following commands:

``` bash
$ cd jetson-inference/tools
$ ./download-models.sh
```

## Installing PyTorch

Le menu suivant s'affichera, appuyez sur la touche `espace` pour séléctionner la version de PyTorch. Appuyez sur `entrer` pour continuer.

<img src="https://raw.githubusercontent.com/dusty-nv/jetson-inference/python/docs/images/pytorch-installer.jpg" width="650">

You can also run this tool again later if you decide that you want to install PyTorch at another time:

``` bash
$ cd jetson-inference/build
$ ./install-pytorch.sh
```

## Compiling the Project

Make sure you are still in the `jetson-inference/build` directory, created above in step #3.

Then run `make` followed by `sudo make install` to build the libraries, Python extension bindings, and code samples:

``` bash
$ make
$ sudo make install
$ sudo ldconfig
```
