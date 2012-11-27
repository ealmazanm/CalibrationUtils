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
#include "KinectSensor.h"
#include "Plane.h"
#include "XnCppWrapper.h"

using namespace xn;
using namespace std;
using namespace cv;


class PlaneFitting
{
public:

	static void recoverPlanes(KinectSensor& kinect, KinectSensor& kinectRef, const XnDepthPixel* depthMap, const XnDepthPixel* depthMapRef, const XnRGB24Pixel* rgbMap, const XnRGB24Pixel* rgbMapRef, Rect& selectWindow, Plane* plane, Plane* planeRef, int& trackingMode, int& numPlanes);

	static int NO_TRACKING;
	static int BUILD_MODEL;
	static int TRACK_MODEL;
	static int COUNTDOWN_MODEL;

};

