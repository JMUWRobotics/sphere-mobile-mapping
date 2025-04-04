# Spherical mobile mapping systems

![prototypeImage](https://github.com/JMUWRobotics/sphere-mobile-mapping/blob/main/delta_pose_filter/img/prototype.jpg?raw=true)

The source code maintained in this repository contains ROS1 packages (running on the prototype shown above with ROS Melodic on Ubuntu 18.04). Included hardware on the platform is:

- Computer: [BMAX Maxmini B3 Plus](https://www.bmaxit.com/Maxmini-B3-Plus-pd722218588.html)
- LiDAR: [Hesai Pandar-XT32](https://www.oxts.com/wp-content/uploads/2021/01/Hesai-PandarXT32_Brochure.pdf)
- Visual-inertial Tracking Camera: [Intel RealSense T265](https://www.intelrealsense.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf?_ga=2.85339993.1922532884.1743763554-46394901.1743498260)
- Inertial Measurement Units: [PhidgetsSpatial Precision 3/3/3 HighRes](https://www.phidgets.com/?prodid=1158#Tab_User_Guide) 

## Prerequisits and Dependencies

### Hesai Pandar-XT32

The LiDAR driver (SDK and ROS wrapper) is included as a catkin package in this repository in the folder [hesai\_lidar\_general](https://github.com/JMUWRobotics/sphere-mobile-mapping/tree/main/hesai_lidar_general).
Dependencies are:

- libpcap-dev
- libyaml-cpp-dev
- python-catkin-tools

Install them all with apt package manager:
```
$ sudo apt install libpcap-dev libyaml-cpp-dev python-catkin-tools
```

### Intel RealSense T265 

The T265 camera ROS wrapper is included as a catkin package in this repository in the folder [vio\_tracking\_t265](https://github.com/JMUWRobotics/sphere-mobile-mapping/tree/main/vio_tracking_t265).
Dependencies are:

- librealsense2 (=2.50 patched)

**Do not** install librealsense2 from the apt package manager! It is likely a version that does not support the T265 camera anymore. Even if you manage to install a legacy version, it likely does not include an important patch!

> Follow [these instructions](vio_tracking_t265/README.md) to compile and install the patched version of librealsense2.   

### PhidgetSpatial IMUs 

The ROS wrapper for the IMUs is included as a catkin package in this repository in the folder [imu\_odom\_phidgets](https://github.com/JMUWRobotics/sphere-mobile-mapping/tree/main/imu_odom_phidgets).  
Dependencies are:

- libphidget22
- libphidget22-dev

Install libphidget22 using the official script:
```
$ sudo apt install curl
$ curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
$ sudo apt update
$ sudo apt install -y libphidget22 libphidget22-dev
```

> If the install script fails, visit [this page](https://www.phidgets.com/docs/OS_-_Linux#Non-Root-2) for alternative methods.

> libphidget22 depends on [glibc version 2.29](https://ftp.gnu.org/gnu/glibc/glibc-2.29.tar.xz) or higher! If you don't meet this requirement (check with ```$ ldd --version```), follow the steps below: 

To compile and install glibc yourself carefully follow these instructions. If not followed carefully **this may break your system** since many things depend on glibc!

1. Download [glibc-2.29](https://ftp.gnu.org/gnu/glibc/glibc-2.29.tar.xz) and extract it
2. A folder called glibc-2.29 should have been extracted. Compile:
```
$ mkdir glibc-2.29/build 
$ cd glibc-2.29/build
$ ../configure --prefix=/opt/glibc
$ make 
```
3. Install the library:
```
$ sudo make install 
```

> You will have to manually patch the executable of the ROS wrapper later, such that it uses the newly installed glibc version. Prepare for that: ```$ sudo apt install patchelf```

## Install 

First step is to [install ROS](https://wiki.ros.org/Distributions#List_of_Distributions).
Preferably ROS Melodic or ROS Noetic (if available). ROS2 is currently not supported. 

Go inside your catkin workspace. If you don't already have one:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
```

Then clone this repository:
```
$ git clone https://github.com/JMUWRobotics/sphere-mobile-mapping.git
```

Build the packages:
```
$ cd ..
$ catkin_make
```

_(Optionally)_ If glibc is manually compiled and installed in /opt/glibc:
```
$ patchelf --set-interpreter /opt/glibc/lib/ld-linux-x86-64.so.2 --set-rpath /opt/glibc/lib:/opt/ros/melodic/lib /home/deadalus/catkin_ws/devel/lib/imu_odom_phidgets/imuJasper
```
## Configuration and Execution 

Use the launch file included in this repository to launch all the nodes.
This files also includes all parameters such as extrinsic parameters. 

``` 
roslaunch ~/catkin_ws/src/sphere.launch
``` 

## Documentation 

Some packages contain their own README file which includes more detailed information. A 3D model (stl file) of the laser-cut assembly parts will follow shortly. 

Contact: fabian.arzberger@uni-wuerzburg.de 
