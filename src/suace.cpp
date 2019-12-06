/*////////////////////////////////////////////////////////////////////////////////////////
//	BSD 2-Clause License

//  Copyright (c) 2017, Ravimal Bandara
//  All rights reserved.

//	Redistribution and use in source and binary forms, with or without
//	modification, are permitted provided that the following conditions are met:

//	* Redistributions of source code must retain the above copyright notice, this
//	list of conditions and the following disclaimer.

//	* Redistributions in binary form must reproduce the above copyright notice,
//	this list of conditions and the following disclaimer in the documentation
//	and/or other materials provided with the distribution.

//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/////////////////////////////////////////////////////////////////////////////////////////

// #include "stdafx.h"
#include "suace.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

void performSUACE(Mat & src, Mat & dst, int distance, double sigma) {

	CV_Assert(src.type() == CV_8UC1);
	if (!(distance > 0 && sigma > 0)) {
		CV_Error(CV_StsBadArg, "distance and sigma must be greater 0");
	}
	dst = Mat(src.size(), CV_8UC1);
	Mat smoothed;
	int val;
	int a, b;
	int adjuster;
	int half_distance = distance / 2;
	double distance_d = distance;

	GaussianBlur(src, smoothed, cv::Size(0, 0), sigma);

	for (int x = 0;x<src.cols;x++)
		for (int y = 0;y < src.rows;y++) {
			val = src.at<uchar>(y, x);
			adjuster = smoothed.at<uchar>(y, x);
			if ((val - adjuster) > distance_d)adjuster += (val - adjuster)*0.5;
			adjuster = adjuster < half_distance ? half_distance : adjuster;
			b = adjuster + half_distance;
			b = b > 255 ? 255 : b;
			a = b - distance;
			a = a < 0 ? 0 : a;

			if (val >= a && val <= b)
			{
				dst.at<uchar>(y, x) = (int)(((val - a) / distance_d) * 255);
			}
			else if (val < a) {
				dst.at<uchar>(y, x) = 0;
			}
			else if (val > b) {
				dst.at<uchar>(y, x) = 255;
			}
		}
}
