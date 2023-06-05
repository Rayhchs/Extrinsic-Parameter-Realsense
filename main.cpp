#include "lib/utils.h"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace Utils;
using namespace std;
using namespace cv;



int sim(){

	static Coord_Cvt Cvt;
	static RawHelper RH;
	static OtherTools OT;
	Parameter Cam1_par;
	Parameter Cam2_par;

	// Read config file
	YAML::Node config = YAML::LoadFile("config.yaml");

	// Camera information provided by realsense
	Cam1_par.scale = 0.001;
	Cam2_par.scale = 0.001;
	OT.yaml2eigen(config, Cam1_par, "Cam1_inter");              	   
	OT.yaml2eigen(config, Cam2_par, "Cam2_inter");  

	// Assume Cam1 at (0, 0, 0)
	Cam1_par.Cam_exter << 1, 0, 0, 0,
			   			  0, 1, 0, 0,
			   			  0, 0, 1, 0,
			   			  0, 0, 0, 1;

	// Load image
	std::string cam1_file, cam2_file;
	cam1_file = config["Cam1_img"].as<std::string>();
	cam2_file = config["Cam2_img"].as<std::string>();

	cv::Mat Cam1_img = cv::imread(cam1_file, cv::IMREAD_UNCHANGED);
	cv::Mat Cam2_img = cv::imread(cam2_file, cv::IMREAD_UNCHANGED);
	cv::Size imageSize = Cam1_img.size();
	std::vector<int> sizeVector = { imageSize.height, imageSize.width };

	// Load depth data of cam1
	std::string cam1_dep;
	cam1_dep = config["Cam1_dep"].as<std::string>();
	vector<float> Cam1_depth_input = RH.ReadRawFile(cam1_dep);
	cv::Mat Cam1_depth;
	RH.Reshape(Cam1_depth_input, sizeVector, Cam1_depth);
	// Cam1_depth *= Cam1_par.scale;

	// Find corner points of each camera
	cv::Size boardSize(5, 8);
    std::vector<cv::Point> C1, C2;
    bool f1 = cv::findChessboardCorners(Cam1_img, boardSize, C1);
    bool f2 = cv::findChessboardCorners(Cam2_img, boardSize, C2);

    if (!f1 || !f2){
    	cout << "Cannot find corner points" << endl;
    	return 1;
    }

    // Find World coordinate of each corner point
    int i;
    MatrixXd Ws(4,40);
    for (i=0; i<Ws.cols(); i++){
    	MatrixXd W(4, 1);
    	Cvt.pixel2world(Cam1_par.Cam_inter, Cam1_par.Cam_exter, C1[i].x, C1[i].y, Cam1_depth.at<float>(C1[i].y, C1[i].x), W);
    	Ws.col(i) = W;
    }
	MatrixXd Ws_extract = Ws.block(0, 0, 3, Ws.cols());

    // Get extrinsic parameter of Cam2
    MatrixXd C2_mat = OT.vectorToPointMatrix(C2);
    Cvt.get_ext(Ws_extract, C2_mat, Cam2_par.Cam_inter, Cam2_par.Cam_exter);
    cout << Cam2_par.Cam_exter << endl;

	return 0;
}


int main(){

	return sim();
}