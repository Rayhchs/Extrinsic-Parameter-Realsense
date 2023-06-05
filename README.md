# Extrinsic Parameter Realsense
This repository displays a implementation of obtaining relative camera extrinsic parameter using RealSense as example.
To compute the relative extrinsic parameters of Cam2 from Cam1, one needs the intrinsic parameters of both cameras, a chessboard shooting by varying perspectives of cameras, and depth data.
The process assumes the initial position of Cam1 at (0, 0, 0).
Firstly, the corner points in the image captured by Cam1 are detected.
These points are then transformed from pixel coordinate to world coordinate utilizing their corresponding (u, v) coordinates and depth information.
Subsequently, the corner points in the image captured by Cam2 are detected.
By providing world coordinate and pixel cooridnate of corner points, the cv2.solvePnP function is employed to calculate the relative extrinsic parameters of Cam2. 

## Setup

* Clone this repository

    ```shell script
    git clone https://github.com/Rayhchs/Extrinsic-Parameter-Realsense.git
    ```

* Environment

Ubuntu 18.04

* Library
1. Eigen
2. OpenCV
3. yaml-cpp

## How to use?

Open config.yaml and change any variable you want.
|    Argument    |                                                                                                       Explanation                                                                                                       |
|:--------------:|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|      Cam1_inter      | Intrinsic parameter of Camera 1 which is assuamed at (0, 0, 0) |
|    Cam2_inter   | Intrinsic parameter of Camera 2 |
|    Cam1_img   | RGB image of Camera1 |
|    Cam2_img   | RGB image of Camera2 |
|    Cam1_dep   | depth image of Camera2 |