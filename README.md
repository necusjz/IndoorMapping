# Indoor Mapping
Generate dense point cloud based on [ORB-SLAM](https://github.com/raulmur/ORB_SLAM2), and build indoor navigation map using OctoMap.

By adding a point cloud viewer, we can visualize dense point cloud during _simultaneous localization and mapping_. We also provide some useful tools in `tools/*`, such as binary dictionary conversion and octree map conversion.

## Installation
There are some prerequisites that need to be installed in advance (all passed on Ubuntu 16.04).

### CMake
[CMake](https://cmake.org/) is an open-source, cross-platform family of tools designed to build, test and package software:
```
$ sudo apt-get install cmake
```

### GLEW
The OpenGL Extension Wrangler Library ([GLEW](http://glew.sourceforge.net/)) is a cross-platform open-source C/C++ extension loading library:
```
$ sudo apt-get install libglew-dev
```

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface:
```
$ git clone https://github.com/stevenlovegrove/Pangolin.git
$ cd Pangolin
$ mkdir build && cd build
$ cmake ..
$ cmake --build .
```

### OpenCV
We use [OpenCV](https://opencv.org/) to manipulate images and features:
```
$ sudo apt-get install build-essential libgtk2.0-dev libvtk5-dev libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev libtbb-dev python-numpy python-matplotlib
$ cd opencv-3.2.0
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```

### Eigen3
[Eigen](http://eigen.tuxfamily.org/) is a C++ template library for linear algebra: matrices, vectors, numerical solvers and related algorithms:
```
$ sudo apt-get install libeigen3-dev
```

### g2o
We use modified version of [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations:
```
$ cd Thirdparty/g2o
$ mkdir build && cd build
$ cmake ..
$ make
``` 

### DBoW2
We use modified version of [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition:
```
$ cd Thirdparty/DBoW2
$ mkdir build && cd build
$ cmake ..
$ make
```

### Boost
[Boost](https://www.boost.org/) provides free peer-reviewed portable C++ source libraries:
```
$ sudo apt-get install libboost-all-dev
```

### FLANN
[FLANN](https://www.cs.ubc.ca/research/flann/) is a library for performing fast approximate nearest neighbor searches in high dimensional spaces:
```
$ sudo apt-get install libflann1.8 libflann-dev
```

### Qt
[Qt](https://www.qt.io/) is the faster, smarter way to create innovative devices, modern UIs & applications for multiple screens:
```
$ chmod +x qt-opensource-linux-x64-5.8.0.run
$ ./qt-opensource-linux-x64-5.8.0.run
```

### VTK
The Visualization Toolkit ([VTK](https://www.vtk.org/)) is an open-source, freely available software system for 3D computer graphics, image processing and visualization:
```
$ cd VTK-8.1.1
$ mkdir build && cd build
$ cmake -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=[PATH]/Qt5.8.0/5.8/gcc_64/bin/qmake -DVTK_Group_Qt:BOOL=ON -DCMAKE_PREFIX_PATH:PATH=[PATH]/Qt5.8.0/5.8/gcc_64/lib/cmake -DBUILD_SHARED_LIBS:BOOL=ON ..
$ make
```

### PCL
The Point Cloud Library ([PCL](http://pointclouds.org/)) is a standalone, large scale, open project for 2D/3D image and point cloud processing:
```
$ cd pcl-pcl-1.8.1
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```

### OctoMap
The [OctoMap](https://octomap.github.io/) library implements a 3D occupancy grid mapping approach, providing data structures and mapping algorithms in C++ particularly suited for robotics:
```
$ sudo apt-get install libqglviewer-dev-qt4
$ git clone https://github.com/OctoMap/octomap.git
$ cd octomap
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```

### Dataset
[Computer Vision Group - Dataset Download](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)

## Usage
```
$ git clone https://github.com/necusjz/IndoorMapping.git
$ cd IndoorMapping
$ mkdir build && cd build
$ cmake ..
$ make
```

## Evaluation
There are several useful commands and we also provide shell script version in `scripts/*.sh`.

### Run test dataset
```
$ ./bin/rgbd_tum Vocabulary/ORBvoc.bin tests/TUM1.yaml Dataset/rgbd_dataset_freiburg1_room tests/association/fr1_room.txt
```

Execute this command, the dense point cloud will be visualized in a viewer like this:
![](https://raw.githubusercontent.com/necusjz/p/master/IndoorMapping/1.jpeg)

### Convert *.pcd to *.ot
```
$ ./tools/pcd2octomap map.pcd map.ot
```

### Plot estimated difference
```
$ ./tools/evaluate_ate.py Dataset/rgbd_dataset_freiburg1_room/groundtruth.txt CameraTrajectory.txt --plot result.png
```

This command will plot _absolute trajectory error_ on a figure like this:
![](https://raw.githubusercontent.com/necusjz/p/master/IndoorMapping/2.jpeg)

### Print all results
```
$ ./tools/evaluate_ate.py Dataset/rgbd_dataset_freiburg1_room/groundtruth.txt CameraTrajectory.txt --verbose
```

## Contributing
We love contributions! Before submitting a Pull Request, it's always good to start with a new issue first.

## License
This repository is licensed under MIT. Full license text is available in [LICENSE](https://github.com/necusjz/IndoorMapping/blob/master/LICENSE).
