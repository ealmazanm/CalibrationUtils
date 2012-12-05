#pragma once
#include <stdio.h>
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"
 #include <XnUSB.h> 

using namespace std;
using namespace cv;
using namespace xn;


#define MAX_DEPTH 10000

class KinectSensor
{
public:
	KinectSensor(void);
	~KinectSensor(void);
	int getIdCam() const;
	int getIdRefCam() const;
	//Initialize all the intrinsic and extrinsic (wrt idRefCam) parameters. "rgbDepth_aligned" alignes rgb and depth image.
	void initDevice(int idCam, int idRefCam, bool rgbDepth_aligned, char* path = NULL);
	void startDevice();
	void stopDevice();
	bool tilt(int angle);
	void shutDown();
	void waitAndUpdate();
	const XnDepthPixel* getDepthMap();
	void getDepthImage(Mat&);
	void getRGBImage(Mat&);
	const XnRGB24Pixel* getRGBMap();
	Point pointProject(const Matx31d& point3D) const;
	Matx31d pointBackproject(const Matx31d& point2D) const;
	Matx31d transformPoint(const Matx31d& point3D) const;
	void transformArray(XnPoint3D*, int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES);
	XnPoint3D* arrayBackProject(const XnPoint3D* depthPonits, int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES) const;
	void createRecorder(Recorder* rec, char* path);
	void releaseRecorder(Recorder* rec);

	XnUInt64 getFocalLength()const;
	XnDouble getPsize() const;
	float getOx() const;
	float getOy() const;

	void setExtrinsics(const Matx33f& rot, const Matx31f& trans);


private:
	
	void initExtrinsics(int id, int idRefCam);
	void open();
	void close();
	void BuildDepthToGreyMapping(unsigned short *pMapping);

	int idCam;
	int idRefCam;
	Context context;
	DepthGenerator depthNode;
	ImageGenerator rgbNode;

	XnUInt64 focalLength;
	float ox, oy;
	XnDouble pSize;
	int tiltAngle;

	Matx33f rotation;
	Matx31f translation;

	//tilt motor
	XN_USB_DEV_HANDLE m_dev;
	bool m_isOpen; 

	unsigned short Mapping[MAX_DEPTH];
};

