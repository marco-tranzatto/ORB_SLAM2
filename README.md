# SERVO

**Original Implementation:** [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**Modified:** Ueli Graf, 07.03.2017

**Current version:** 1.0.0 

This is SERVO, a robust SLAM extension to ORB-SLAM2. SERVO relies on additional odometry estimates provided in a ROS environment. At the time of writing, only stereo SLAM mode is supported.

#1. Quick start

##1.1 Install dependencies:

[ROS](ros.org)

[Pangolin](https://github.com/stevenlovegrove/Pangolin)

[OpenCV](http://opencv.org) **Required at leat 2.4.3. Tested with OpenCV 2.4.11**

[Eigen3](http://eigen.tuxfamily.org) **Required at least 3.1.0**

BLAS: 2
```
sudo apt-get install libblas-dev
```
LAPACK: 
```
sudo apt-get install liblapack-dev
```

##1.2 Install additional odometry software
This software was tested with [rovio](https://github.com/ethz-asl/rovio) which is included as a submodule. 

##1.3 Clone and build RSLAM

Clone the repository and execute the build script found in the root directory at SERVO/build.sh 
```
git clone https://github.com/grafue/SERVO.git
cd SERVO
chmod +x build.sh
./build.sh
```
##1.4 Build catkin packages
```
catkin build
```

##1.5 Join the rosbag provided for testing as a split archive

Open a terminal at SERVO/Examples/ROS/
```
cat SERVO_bag_* > SERVO.bag
```

##2 Launch
Use the launch file found at SERVO/Examples/ROS/launch/servo.launch for an exemplary launch file.

The current version assumes that rovio is used in parallel (that is, rovio must be running) and publishes odometry estimates to /rovio/odometry and the transformation from camera frame to odometry reference frame to /rovio/extrinsics0.
rosbag can be selected within the launch file. If the resulting estimated trajectory is implausible, try decrease the rate (-r parameter in the rosbag section in servo.launch).
Topics can be changed by changing the remap command in SERVO.launch or by changing the source code at ros_stereo.cc.

```
roslaunch Examples/ROS/launch/servo.launch
```

#3 Original Documentation:
For extensive documentation on the base implementations of the frameworks used, see the original repositories of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) and [rovio](https://github.com/ethz-asl/rovio).



