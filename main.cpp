#include "lib/utils.h"
#include <iostream>
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

	// Camera information provided by realsense
	Cam1_par.scale = 0.001;
	Cam1_par.Cam_inter << 902.847, 0.0, 650.313,
		              	  0.0, 902.784, 359.130,
		              	  0.0, 0.0, 1.0;
	Cam2_par.scale = 0.001;              	   
	Cam2_par.Cam_inter << 907.792, 0.0, 645.128,
		              	  0.0, 908.006, 362.454,
		              	  0.0, 0.0, 1.0;

	// Assume Cam1 at (0, 0, 0)
	Cam1_par.Cam_exter << 1, 0, 0, 0,
			   			  0, 1, 0, 0,
			   			  0, 0, 1, 0,
			   			  0, 0, 0, 1;

	// Load image
	cv::Mat Cam1_img = cv::imread("Cam1/cam.png_Color_1683105025876.77001953125000.png", cv::IMREAD_UNCHANGED);
	cv::Mat Cam2_img = cv::imread("Cam2/eye.png_Color_1683098027550.76708984375000.png", cv::IMREAD_UNCHANGED);
	cv::Size imageSize = Cam1_img.size();
	std::vector<int> sizeVector = { imageSize.height, imageSize.width };

	// Load depth data of cam1
	string filename = "Cam1/cam.raw_Depth_1683105025868.10009765625000.raw";
	vector<float> Cam1_depth_input = RH.ReadRawFile(filename);
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