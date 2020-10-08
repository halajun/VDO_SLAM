# VDO-SLAM
**Authors:** [Jun Zhang](https://halajun.github.io/), [Mina Henein](https://minahenein.github.io/), [Robert Mahony](https://users.cecs.anu.edu.au/~Robert.Mahony/) and [Viorela Ila](http://viorelaila.net/) 

VDO-SLAM is a Visual Object-aware Dynamic SLAM library for **RGB-D** cameras that is able to track dynamic objects, estimate the camera poses along with the static and dynamic structure, the full SE(3) pose change of every rigid object in the scene, extract velocity information, and be demonstrable in real-world outdoor scenarios. We provide examples to run the SLAM system in the [KITTI Tracking Dataset](http://cvlibs.net/datasets/kitti/eval_tracking.php), and in the [Oxford Multi-motion Dataset](https://robotic-esp.com/datasets/omd/). 


# 1. Prerequisites
We have tested the library in **Mac OS X 10.14** and **Ubuntu 16.04**, but it should be easy to compile in other platforms. 

## c++11, gcc and clang
We use some functionalities of c++11, and the tested gcc version is 9.2.1 (ubuntu), the tested clang version is 1000.11.45.5 (Mac).

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Download and install instructions can be found at: http://opencv.org. **Required at least 3.0. Tested with OpenCV 3.4**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## g2o (Included in dependencies folder)
We use modified versions of [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. The modified libraries (which are BSD) are included in the *dependencies* folder.


# 2. Building Library

In the source code folder, we provide a script `build.sh` to build the *dependencies* libraries and *VDO-SLAM*. 
Please make sure you have installed all required dependencies (see Section 1). 
Please also change the library file suffix, i.e., '.dylib' for Mac (default) or '.so' for Ubuntu, in the main CMakeLists.txt. 
Then Execute:
```
cd VDO-SLAM
chmod +x build.sh
./build.sh
```

This will create 

1. **libObjSLAM.dylib (Mac)** or **libObjSLAM.so (Ubuntu)** at *lib* folder,

2. **libg2o.dylib (Mac)** or **libg2o.so (Ubuntu)** at */dependencies/g2o/lib* folder,

3. and the executable **vdo_slam** in *example* folder.

# 3. Running Examples

## KITTI Tracking Dataset  

1. Download the demo sequence: [kitti_demo](https://drive.google.com/file/d/1LpjIdh6xL_UtWOkiJng0CKSmP7qAQhGu/view?usp=sharing), and uncompress it.

2. Execute the following command.
```
./example/vdo_slam example/kitti-0000-0013.yaml PATH_TO_KITTI_SEQUENCE_DATA_FOLDER
```

## Oxford Multi-motion Dataset  

1. Download the demo sequence: [omd_demo](https://drive.google.com/file/d/1t4rG685a_7r0bHuW0bPKNbOhyiugnJK7/view?usp=sharing), and uncompress it.

2. Execute the following command.
```
./example/vdo_slam example/omd.yaml PATH_TO_OMD_SEQUENCE_DATA_FOLDER
```







