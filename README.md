# Landmark Slam

This repo contains the implementation of several SLAM algorithms, such as:

* EKF SLAM
* UKF SLAM (TODO)
* EIF SLAM (TODO)
* Fast SLAM

Implementation was done in C++ using the Eigen3 library.

## Install

Build the executeable ```slam``` using the CMake build system:

```bash
cd "path/to/repo"
mkdir build
cd build
cmake ..
make
```

You can then run the application using

```bash
./slam
```

No command line options are currently supported. The algorithm has to be chosen in the source code manually. This requires a recompilation.
