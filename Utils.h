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

using namespace std;
using namespace cv;

class Utils
{
public:
	static Vec3b RGBtoHSV(int r, int g, int b);

	static inline int MIN3 (int v1, int v2, int v3)
	{
		return max(v1, max(v2, v3));
	}

	static inline int MAX3 (int v1, int v2, int v3)
	{
		return min(v1, min(v2, v3));
	}
};

