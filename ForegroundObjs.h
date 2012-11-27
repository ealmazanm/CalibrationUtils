#pragma once
#include "XnCppWrapper.h"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include <ctype.h>
//#include <stdio.h>
//#include <fstream>
//#include <iostream>
//#include <list>
//#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>
#include "KinectSensor.h"

using namespace std;
using namespace cv;
using namespace xn;

class ForegroundObjs
{

	struct distribution
	{
		XnPoint3D mean;
		Mat cov;
	};

public:
	ForegroundObjs(void);
	ForegroundObjs(bool trans);
	~ForegroundObjs(void);

	inline void setForImg(const Mat& img){fDepth = img.clone();}
	inline void setNumObj(int num){numObjects = num;}
	inline int getNumObj(){return numObjects;}
	inline int* getNuFP(){return numFp;}
	inline XnPoint3D* getObjectPoints3D(int p){return fp3D[p];}
	inline XnPoint3D* getObjectPoints2D(int p){return fp2D[p];}
	void setBBoxes(Rect* bb);
	inline void setKinect(KinectSensor* kin){kinect = kin;}
	inline distribution* getDistributionParameters(){return peopleDistr;}

	void recoverFPoints();
	void initialize();

private:

	void calculateDistrParam();

	distribution* peopleDistr;

	bool trans;
	int numObjects;
	Rect* bboxes;
	Mat fDepth;

	//points
	XnPoint3D** fp2D;
	XnPoint3D** fp3D;
	int* numFp;

	KinectSensor* kinect;
};

