#pragma once
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <ctype.h>
#include <fstream>
#include "XnCppWrapper.h"
#define MAX_DEPTH 10000

using namespace xn;
using namespace std;
using namespace cv;

class Utils
{
public:
	static Vec3b RGBtoHSV(int r, int g, int b);

	static void combineTwoImages(const Mat* Frame1, const Mat* Frame2, Mat& Frame12);

	static void initMatf(Mat&, int);

	static void initMat3u(Mat&, int);

	static void convertXnRGB24PixelToFrame(const XnRGB24Pixel*,Mat& M);

	static void convertXnDepthPixelToFrame(const XnDepthPixel *P, Mat& M);

	static inline int MIN3 (int v1, int v2, int v3)
	{
		return max(v1, max(v2, v3));
	}

	static inline int MAX3 (int v1, int v2, int v3)
	{
		return min(v1, min(v2, v3));
	}
};

