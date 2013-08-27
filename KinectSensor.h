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
	void turnUpsideDown(XnDepthPixel* dMap_ud, XnRGB24Pixel* rgbMap_ud);
	void getDepthImage(Mat&);
	void getDepthImage(Mat& dImage, const XnDepthPixel* dMap);
	void getRGBImage(Mat&);
	void getRGBImage(Mat& rgbImage, const XnRGB24Pixel* rgbMap);
	const XnRGB24Pixel* getRGBMap();
	Point pointProject(const Matx31d& point3D) const;
	Matx31d pointBackproject(const Matx31d& point2D) const;
	Matx31d transformPoint(const Matx31d& point3D) const;
	void transformArray(XnPoint3D*, Mat& outPoints, int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES);
	void transformArray(XnPoint3D*, int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES);
	XnPoint3D* arrayBackProject(const XnPoint3D* depthPonits, int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES) const;
	XnPoint3D* arrayProject(const XnPoint3D* realPoints,int numPoints = XN_VGA_X_RES*XN_VGA_Y_RES) const;
	void createRecorder(Recorder* rec, char* path);
	void releaseRecorder(Recorder* rec);
	int getNumberFrames();
	int getFrameID();
	inline int getTilt(){return tiltAngle;}

	XnUInt64 getFocalLength()const;
	XnDouble getPsize() const;
	float getOx() const;
	float getOy() const;

	void setExtrinsics(const Matx33f& rot, const Matx31f& trans);

	Matx33f rotation;
	Matx31f translation;
	Matx33f rotationInv;
	Matx31f translationInv;

private:
	
	void initExtrinsics(int id, int idRefCam);
	void open();
	void close();
	void BuildDepthToGreyMapping(unsigned short *pMapping);
	void transformPointByPoint(XnPoint3D* points, int numPoints);
	void transformArrayPoints(XnPoint3D* points, Mat& outPoints, int numPoints);

	Matx33f rotTilt;
	int idCam;
	int idRefCam;
	Context context;
	DepthGenerator depthNode;
	ImageGenerator rgbNode;

	XnUInt64 focalLength;
	float ox, oy;
	XnDouble pSize;
	int tiltAngle;



	//tilt motor
	XN_USB_DEV_HANDLE m_dev;
	bool m_isOpen; 

	unsigned short Mapping[MAX_DEPTH];

	XnUInt32 uFrames;
};

