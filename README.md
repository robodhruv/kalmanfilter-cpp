Kalman Filter on C++ (Eigen)
=============
This is a basic Kalman filter implementation in C++ using the
[Eigen](http://eigen.tuxfamily.org/) library. It implements the algorithm
directly as found in [An Introduction to the Kalman Filter]
(https://courses.cs.washington.edu/courses/cse571/03wi/notes/welch-bishop-tutorial.pdf) or
demonstrated in this [YouTube video](https://www.youtube.com/watch?v=VFXf1lIZ3p8).

The overall structure of the code has been borrowed from [hmartiro's implementation](https://github.com/hmartiro/kalman-cpp),
extending it to allow input control.

There is a test program that estimates the motion of a projectile based on
noisy observations. To run it, use CMake:

    cd kalmanfilter-cpp
    mkdir build
    cd build
    cmake ..
    make -j4
    ./kalman-test

Note: You may have to specify the path to your Eigen library in `CMakeLists.txt`.