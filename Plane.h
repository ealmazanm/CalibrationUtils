#pragma once
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ctype.h>
#include "KinectSensor.h"
#include "Utils.h"
#include "XnCppWrapper.h"

using namespace std;
using namespace cv;

class Plane
{
public:
	Plane(void);
	~Plane(void);
		
	int	frontoPlaneFit(const XnDepthPixel*, const KinectSensor*, Rect);
	int	updatePlaneFit(const XnDepthPixel*, KinectSensor*, const XnRGB24Pixel*);
	const Matx31d* getParameters() const;
	const Matx31d* getCentroid() const;
	const Matx31d* getNormal() const;
	double getDistance() const;

	void setParameters(Matx31d*);
	void setCentroid(Matx31d*);
	void setNormal(Matx31d*);
	
	Rect getFitWindow() const;
	void setResidualVariance(double);
	double getResidualVariance() const;

	void setFittingWindow(Point p, int width, int height);

private:
	double TukeyBiweight(double e, double c) const;

	Matx31d parameters;	// (a,b,c) where z=ax+by+c
	Matx31d centroid;	// 3D centroid of data
	Matx31d normal;		// 3D normal (towards camera) of plane

	Rect	fitWindow;	// 2D image window containing plane
	double	residualVariance; // variance of residuals e=z-ax-by-c
	// Plane variables 
	float minDepth, maxDepth;
};

