#pragma once
#include <opencv2/opencv.hpp>

using namespace cv;

/*
@brief Enhance the contrast with SUACE;
@param src source image with the type CV_8UC1.
@param dst output image
@param distance the old reference contrast range to be enhanced. less value means more contrast.
@param sigma the sigma value of the Gaussina filter in SUACE
*/
void performSUACE(Mat & src, Mat & dst, int distance=20, double sigma=7);
