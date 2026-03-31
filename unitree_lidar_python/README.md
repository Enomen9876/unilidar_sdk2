This is a C++ SDK for Unitree Lidar L2.

This code is created with Google Gemini!

in build folder run:

cmake .. && make -j2

after successful build -> the python pyckage will be in python_examples folder

there you can run these three examples

> python3 lidar_transformations_depth.py # this is with point retention with imu transformations

> python3 lidar_transformations.py # this is imu transformed but without retention
> python3 lidar_test.py # this is bare lidar scan