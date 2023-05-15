# Extrinsic Parameter Realsense
This repository displays a implementation of obtaining camera extrinsic parameter by using RealSense as example. To compute the relative extrinsic parameters of Cam2, one needs the intrinsic parameters of both cameras, a chessboard shooting by varying perspectives of cameras, and depth data. The process assumes the initial position of Cam1 at (0, 0, 0). Firstly, corner points are computed 