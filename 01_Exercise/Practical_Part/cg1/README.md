# Computer Graphics 1 - Exercises

This repository contains the source code and data sets for the CG1 exercises. To download and compile the project, you need to do the following steps:

1. Clone the project to your local computer
2. Run CMake to generate a project file for your build platform. Alternatively, you can use a development environment that supports CMake natively and open the directory directly.
3. Open the project file and compile.

## Unix Instructions

Under Unix, you could employ the command line like follows:
```bash
git clone https://bitbucket.org/cgvtud/cg1.git --recursive
cd cg1
mkdir build
cd build
cmake ..
make
```

After this, you find the compiled applications under `cg1/build/bin`.

If you are missing some dependencies you can install them on Debian using:
```bash
sudo apt install git make cmake libxxf86vm1 libxrandr2 libxinerama1 libxcursor1 libx11-6 libc6 libstdc++6 libgcc-8-dev libxext6	libxrender1 libxfixes3 libxcb1 libxau6 libxdmcp6 libbsd0
```
Note: This list may look long but most of these packages are likely already installed on your system.

## Windows Instructions
	
On Windows / Visual Studio, you can use the CMake GUI. After cloning the repository, open the CMake GUI and set *Where is the source code* to the cloned directory.
Set *Where to build the binaries* to a new subfolder `build`.
Click *Configure* and select the desired Visual Studio Version. Then, click *Configure*  and *Open Project* to generate and open the project in Visual studio.
Finally, choose the desired startup project (Exercise1-4) and compile and run.

