#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace Eigen;

namespace Utils{

struct Parameter{

    // Depth scale
    float scale;

    // Camera intrinsic parameter
    Eigen::Matrix3d Cam_inter;

    // Camera extrinsic parameter
    Eigen::Matrix4d Cam_exter;

};

class Coord_Cvt{

public:
    void pixel2world(Eigen::Matrix3d interparam, Eigen::Matrix4d exterparam, float u, float v, float depth, Eigen::MatrixXd& world_coords);
    void world2pixel(Eigen::Matrix3d interparam, Eigen::Matrix4d exterparam, Eigen::MatrixXd world_coords, Eigen::MatrixXd& pix_coords);
    void get_ext(Eigen::MatrixXd W, Eigen::MatrixXd E, Eigen::Matrix3d E_int, Eigen::Matrix4d& E_ext);
};

class RawHelper{

public:
    vector<float> ReadRawFile(std::string& filename);
    int Reshape(std::vector<float>& input, std::vector<int>& shape, cv::Mat& output);
};

class OtherTools{

public:
    MatrixXd vectorToPointMatrix(vector<cv::Point> points);
};
}
#endif