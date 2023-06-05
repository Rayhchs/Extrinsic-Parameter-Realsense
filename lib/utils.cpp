#include "utils.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>


using namespace Eigen;
using namespace std;
using namespace cv;
using namespace Utils;


void Coord_Cvt::pixel2world(Matrix3d interparam, Matrix4d exterparam, float u, float v, float depth, MatrixXd& world_coords)
{

	// Pixel coordinate
	float Zc = 1;
	float Xc = u;
	float Yc = v;
	MatrixXd pixel(3, 1);
	pixel << u, v, 1;

	// Calculate inverse matrix of extrinsic and intrinsic parameters
	Matrix3d K_inv = interparam.inverse();
	Matrix4d RT_inv = exterparam.inverse();

	// Calculate from pixel coordinate to cam coordinate
	MatrixXd P_cam = K_inv * pixel * depth;
    MatrixXd value(1, 1);
    value << 1;
    MatrixXd P_cam_stack(4,1);
    P_cam_stack << P_cam, value;

    // Calculate from cam coordinate to pixel coordinate
    world_coords = RT_inv * P_cam_stack;

}


void Coord_Cvt::world2pixel(Matrix3d interparam, Matrix4d exterparam, MatrixXd world_coords, MatrixXd& pix_coords)
{

	Eigen::MatrixXd Cam_ext_top = exterparam.topRows(3);
	pix_coords = interparam * Cam_ext_top * world_coords;

}


void Coord_Cvt::get_ext(MatrixXd W, MatrixXd E, Matrix3d E_int, Matrix4d& E_ext)
{

	// Declaration
	cv::Mat rvec, tvec, R;
	cv::Mat W_mat, W_mat_trans, E_mat, E_int_mat, E_ext_mat;
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
	cv::Mat M = (cv::Mat_<double>(1, 4) << 0., 0., 0., 1.);

	// Transform eigen matrix to cv mat
	cv::eigen2cv(W, W_mat);
    cv::transpose(W_mat, W_mat_trans);
	cv::eigen2cv(E, E_mat);
	cv::eigen2cv(E_int, E_int_mat);
	
	// Calculate extrinsics of eye
	cv::solvePnP(W_mat_trans, E_mat, E_int_mat, distCoeffs, rvec, tvec, false);
	cv::Rodrigues(rvec, R);
	cv::hconcat(R, tvec, E_ext_mat);
	cv::vconcat(E_ext_mat, M, E_ext_mat);
	cv::cv2eigen(E_ext_mat, E_ext);
}


std::vector<float> RawHelper::ReadRawFile(std::string& fileName)
{
    // Open binary file
    std::ifstream inputFile(fileName, std::ios::binary);

    if (!inputFile.is_open())
    {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return {};
    }

    // Allocate vectors
    std::vector<float> result;

    // Calculate bytes
    int numBytesPerRead = 1024 * 1024; // 每次讀取1 MB

    // Allocate buffe
    std::vector<char> buffer(numBytesPerRead);

    while (!inputFile.eof())
    {
        // Read data to buffer
        inputFile.read(buffer.data(), numBytesPerRead);

        if (inputFile.fail() && !inputFile.eof())
        {
            std::cerr << "Failed to read file: " << fileName << std::endl;
            return {};
        }

        // Calculate number of elements
        int numElements = inputFile.gcount() / sizeof(uint16_t);

        // Convert content in buffer to vectors
        auto start = reinterpret_cast<uint16_t*>(buffer.data());
        auto end = start + numElements;
        std::vector<float> temp(end - start);
        std::transform(start, end, temp.begin(), [](uint16_t val){ return val * 0.001f; });
        result.insert(result.end(), temp.begin(), temp.end());
    }

    return result;
}


int RawHelper::Reshape(std::vector<float>& input, std::vector<int>& shape, cv::Mat& output)
{
    if (input.size() != std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int>()))
    {
        std::cerr << "Error: input size does not match target shape" << std::endl;
        return 0;
    }

    cv::Mat input_mat(input);
    output = input_mat.reshape(0, shape);
    return 0;
}


MatrixXd OtherTools::vectorToPointMatrix(vector<cv::Point> points)
{
    int rows = points.size();
    MatrixXd result(rows, 2);
    Map<MatrixXd> map(result.data(), rows, 2);

    for (int i = 0; i < rows; i++) {
        map(i, 0) = points[i].x;
        map(i, 1) = points[i].y;
    }
    return result;
}


int OtherTools::yaml2eigen(const YAML::Node& config, Parameter& params, std::string Cam_name)
{
    for (int i = 0; i < params.Cam_inter.rows(); ++i) {
        for (int j = 0; j < params.Cam_inter.cols(); ++j) {
            params.Cam_inter(i, j) = config[Cam_name][i][j].as<float>();
        }
    }
    return 0;
}