#include "KinectSensor.h"

float KinectSensor::KINECT_HORIZ_FOV = 0.5446;

KinectSensor::KinectSensor(void)
{
	m_isOpen = false;
	BuildDepthToGreyMapping(Mapping);
	tiltAngle = 0;
	uFrames = -1;
}


KinectSensor::~KinectSensor(void)
{
}

int KinectSensor::getIdRefCam() const
{
	return idRefCam;
}
int KinectSensor::getIdCam() const
{
	return idCam;
}

void KinectSensor::setExtrinsics(const Matx33f& rot, const Matx31f& trans)
{
	rotation = rot;
	translation = trans;
}
	
void KinectSensor::initDevice(int id, int idRefC, bool aligned, char* path)
{

	idCam = id;
	idRefCam = idRefC;
	// Initialise sensors
	context.Init();
	
	if (path != NULL)
	{
		context.OpenFileRecording(path);
		context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthNode);
		context.FindExistingNode(XN_NODE_TYPE_IMAGE, rgbNode);

		//Player xPlayer;
		//context.OpenFileRecording(path, xPlayer);
		//xPlayer.SetRepeat(false);
		//context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthNode);
		//context.FindExistingNode(XN_NODE_TYPE_IMAGE, rgbNode);
		//xPlayer.GetNumFrames(depthNode.GetName(), uFrames);
	}
	else
	{
		NodeInfoList depth_node_info_list, image_node_info_list;
		Query query;
		query.SetVendor("PrimeSense");
		context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, &query, depth_node_info_list, NULL);
		context.EnumerateProductionTrees(XN_NODE_TYPE_IMAGE, &query, image_node_info_list, NULL);

		//depth & RGB node instantiation
		NodeInfoList::Iterator dNodeIt = depth_node_info_list.Begin();
		NodeInfoList::Iterator rgbNodeIt = image_node_info_list.Begin();
		int contNodes = 0;
		NodeInfo& depth_Node = *dNodeIt;
		NodeInfo& rgb_Node = *rgbNodeIt;
		while (contNodes <= idCam)
		{
			depth_Node = *dNodeIt++; 
			rgb_Node = *rgbNodeIt++;
			contNodes++;
		}
		depth_Node.GetInstance(depthNode);
		context.CreateProductionTree(depth_Node);
		depthNode.Create(context);

		rgb_Node.GetInstance(rgbNode);
		context.CreateProductionTree(rgb_Node);
		rgbNode.Create(context);
	}
	if (aligned && depthNode.IsCapabilitySupported("AlternativeViewPoint"))
		depthNode.GetAlternativeViewPointCap().SetViewPoint(rgbNode);

	// Initialise Camera with Intrinsics
	depthNode.GetRealProperty ("ZPPS", pSize);
	pSize *= 2.0; //the resolution is reduced due to the bandwidth
	depthNode.GetIntProperty ("ZPD",focalLength);
	ox = XN_VGA_X_RES/2;
	oy = XN_VGA_Y_RES/2;
	double depth_focal_length_VGA_ = (double)focalLength/pSize;

	XnUInt64 zeroPlaneD;
	depthNode.GetIntProperty("ZPD", zeroPlaneD); 
	XnDouble baseline;	
	depthNode.GetRealProperty ("LDDIS", baseline);
	

	
	
   

	// Initialise Camera with Extrinsics
	initExtrinsics(id, idRefC);
}

int KinectSensor::getNumberFrames()
{
	return uFrames;
}

int KinectSensor::getFrameID()
{
	return depthNode.GetFrameID();
}
	
void KinectSensor::startDevice()
{
	context.StartGeneratingAll();
}
	
void KinectSensor::stopDevice()
{
	context.StopGeneratingAll();
}

XnPoint3D* KinectSensor::arrayProject(const XnPoint3D* realPoints, int numPoints) const
{
	XnPoint3D* imagePoints = new XnPoint3D[numPoints];
	depthNode.ConvertRealWorldToProjective(numPoints, realPoints, imagePoints);
	return imagePoints;
}

XnPoint3D* KinectSensor::arrayBackProject(const XnPoint3D* depthPoints, int numPoints) const
{
	XnPoint3D* realPoints = new XnPoint3D[numPoints];
	depthNode.ConvertProjectiveToRealWorld(numPoints, depthPoints, realPoints);
	return realPoints;
}

/*
Returns the Y axis inverted
*/
Point KinectSensor::pointProject(const Matx31d& point3D) const
{
	XnPoint3D realPoint;
	realPoint.X = point3D(0); realPoint.Y = point3D(1); realPoint.Z = point3D(2);
	XnPoint3D camPoint;
	depthNode.ConvertRealWorldToProjective(1, &realPoint, &camPoint);
	Point p;
	p.x = camPoint.X;
	p.y = camPoint.Y;
	//return p;


	Point P1;
	P1.x = (int) (ox + (point3D.val[0]/point3D.val[2])*(focalLength/pSize));
	P1.y = (int) (oy + (point3D.val[1]/point3D.val[2])*(focalLength/pSize));
	return P1;
}

//turn upside down the image
void KinectSensor::turnUpsideDown(XnDepthPixel* dMap_ud, XnRGB24Pixel* rgbMap_ud)
{
	const XnDepthPixel* dMap = depthNode.GetDepthMap();
	const XnRGB24Pixel* rgbMap = rgbNode.GetRGB24ImageMap();
	//TODO: turn the arrays upside down
	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		for (int x = 0; x < XN_VGA_X_RES; x++)
		{
			dMap_ud[y*XN_VGA_X_RES+x] = dMap[y*XN_VGA_X_RES + (XN_VGA_X_RES-1-x)];
			rgbMap_ud[y*XN_VGA_X_RES+x] = rgbMap[y*XN_VGA_X_RES + (XN_VGA_X_RES-1-x)];
		}
	}
}

/*
Returns the Y axis inverted
*/
Matx31d KinectSensor::pointBackproject(const Matx31d& point2D) const
{
	XnPoint3D realPoint;
	XnPoint3D camPoint;
	camPoint.X = point2D(0);
	camPoint.Y = point2D(1);
	camPoint.Z = point2D(2);

	depthNode.ConvertProjectiveToRealWorld(1, &camPoint, &realPoint);
	Matx31d out (realPoint.X, realPoint.Y, realPoint.Z);
	//return out;


	Matx31d p3d;
	p3d(0) = (point2D(0) - ox)*pSize*point2D(2)/focalLength;
	p3d(1) = (point2D(1) - oy)*pSize*point2D(2)/focalLength;
	p3d(2) = point2D(2);
	return p3d;
}

void KinectSensor::transformPointByPoint(XnPoint3D* points, int numPoints)
{
	if (idCam != idRefCam)
	{
		for (int i = 0; i < numPoints; i++)
		{
			Matx31f p(points[i].X, points[i].Y, points[i].Z);
			Matx31f out = rotation*p+translation;

			if (tiltAngle != 0)	
			{
				out = (out.t() * rotTilt).t();
			}
		
			points[i].X = out(0);
			points[i].Y = out(1);
			points[i].Z = out(2);
		}
	}
	else if(tiltAngle != 0)
	{
		for (int i = 0; i < numPoints; i++)
		{
			Matx31f p(points[i].X, points[i].Y, points[i].Z);
			Matx13f out = p.t()*rotTilt;

			points[i].X = out(0);
			points[i].Y = out(1);
			points[i].Z = out(2);
		}
	}
}





void KinectSensor::transformArrayPoints(XnPoint3D* points, Mat& outPoints, int numPoints)
{
	//Mat pointsMat = Mat(numPoints, 3, CV_32F);
	Mat translationExt = Mat(numPoints, 3, CV_32F);

	float* outPoints_data = (float*)outPoints.data;
	int outPoints_step = outPoints.step/sizeof(float);

	float* translation_data = (float*)translationExt.data;
	int translation_step = translationExt.step/sizeof(float);

	float tx = translation(0);
	float ty = translation(1);
	float tz = translation(2);

	for (int i = 0; i < numPoints; i++)
	{ 
		//create points matrix
		float* ptr = outPoints_data + i*outPoints_step;
		XnPoint3D* p = points + i;
		*ptr++ = p->X;
		*ptr++ = p->Y;
		*ptr = p->Z;
		//float* ptr = outPoints.ptr<float>(i);
		//*ptr++ = points[i].X;
		//*ptr++ = points[i].Y;
		//*ptr = points[i].Z;

		//create translation matrix
		float* ptrT = translation_data + i*translation_step;
		*ptrT++ = tx;
		*ptrT++ = ty;
		*ptrT = tz;

	/*	float* ptrT = translationExt.ptr<float>(i);
		*ptrT++ = translation(0);
		*ptrT++ = translation(1);
		*ptrT = translation(2);*/
	}
	if (idCam != idRefCam)
	{
		outPoints = ((Mat)rotation*outPoints.t()+translationExt.t()).t();
	}

	 if(tiltAngle != 0)
	 {
		// outPoints = ((Mat)rotTilt*outPoints.t()).t();
		  outPoints = outPoints*(Mat)rotTilt;
	 }

}


//Transform the array points into a common coordinate system defined by idRefCam
void KinectSensor::transformArray(XnPoint3D* points, Mat& outPoints, int numPoints)
{
	transformArrayPoints(points, outPoints, numPoints);

	
}

void KinectSensor::transformArrayNoTilt_rev(XnPoint3D* points, int numPoints, int side)
{
	char *pathRot_common = "D:\\CameraCalibrations\\extrinsics\\rotation_1";
	char *pathTra_common = "D:\\CameraCalibrations\\extrinsics\\translation_1";
	char pathRot[150];
	char pathTra[150];
	std::strcpy(pathRot, pathRot_common);
	std::strcpy(pathTra, pathTra_common);
	if (idCam == 1)
	{
		if (side == 0) //kinect 1
		{
			std::strcat(pathRot, "0.xml");
			std::strcat(pathTra, "0.xml");
		}
		else //kinect 2
		{
			std::strcat(pathRot, "2.xml");
			std::strcat(pathTra, "2.xml");
		}
	}
	Mat t,r;
	FileStorage fsRot(pathRot, FileStorage::READ);
	FileStorage fsTra(pathTra, FileStorage::READ);
	if (fsRot.isOpened() && fsTra.isOpened())
	{
		
		fsRot["Rotation"] >> r;
		fsTra["Translation"] >> t;
	}

	//Create extended matrices
	Mat pointsMat = Mat(numPoints, 3, CV_32F);
	float* ptrP = (float*)pointsMat.data;
	int stepP = pointsMat.step/sizeof(float);

	Mat translationExt = Mat(numPoints, 3, CV_32F);
	float* ptrT = (float*)translationExt.data;
	int stepT = translationExt.step/sizeof(float);

	float tx = *t.ptr<float>(0);
	float ty = *t.ptr<float>(1);
	float tz = *t.ptr<float>(2);

	for (int i = 0; i < numPoints; i++)
	{ 
		//create points matrix
		float* ptrPoints = ptrP + (i*stepP);
		*ptrPoints++ = points[i].X;
		*ptrPoints++ = points[i].Y;
		*ptrPoints = points[i].Z;

		//create translation matrix
		float* ptrTrans = ptrT + (i*stepT);
		*ptrTrans++ = tx;
		*ptrTrans++ = ty;
		*ptrTrans = tz;
	}

	pointsMat = ((Mat)r*pointsMat.t()+translationExt.t()).t();

	for (int i = 0; i < numPoints; i++)
	{
		points[i].X = (ptrP + (i*stepP))[0];
		points[i].Y = (ptrP + (i*stepP))[1];
		points[i].Z = (ptrP + (i*stepP))[2];
	}

}

void KinectSensor::transformArrayNoTilt(XnPoint3D* points, int numPoints)
{
	if (idCam != idRefCam)
	{
		//Create extended matrices
		Mat pointsMat = Mat(numPoints, 3, CV_32F);
		float* ptrP = (float*)pointsMat.data;
		int stepP = pointsMat.step/sizeof(float);

		Mat translationExt = Mat(numPoints, 3, CV_32F);
		float* ptrT = (float*)translationExt.data;
		int stepT = translationExt.step/sizeof(float);

		float tx = translation(0);
		float ty = translation(1);
		float tz = translation(2);
		for (int i = 0; i < numPoints; i++)
		{ 
			//create points matrix
			float* ptrPoints = ptrP + (i*stepP);
			*ptrPoints++ = points[i].X;
			*ptrPoints++ = points[i].Y;
			*ptrPoints = points[i].Z;

			//create translation matrix
			float* ptrTrans = ptrT + (i*stepT);
			*ptrTrans++ = tx;
			*ptrTrans++ = ty;
			*ptrTrans = tz;
		}

	//	if (idCam == 0)
	//		pointsMat =   pointsMat*(Mat)rotation + translationExt;// ((Mat)rotation*pointsMat.t()+translationExt.t()).t();
	//	else
			pointsMat = ((Mat)rotation*pointsMat.t()+translationExt.t()).t();

		//pointsMat = ((Mat)rotation*pointsMat.t()+translationExt.t()).t();

		for (int i = 0; i < numPoints; i++)
		{
			points[i].X = (ptrP + (i*stepP))[0];
			points[i].Y = (ptrP + (i*stepP))[1];
			points[i].Z = (ptrP + (i*stepP))[2];
		}

	}
}

void KinectSensor::correctTilting(XnPoint3D* points, int numPoints)
{
	//Create extended matrices
	Mat pointsMat = Mat(numPoints, 3, CV_32F);
	float* ptrP = (float*)pointsMat.data;
	int step = pointsMat.step/sizeof(float);

	for (int i = 0; i < numPoints; i++)
	{ 
		//create points matrix
		float* ptrPoints = ptrP + (i*step);
		*ptrPoints++ = points[i].X;
		*ptrPoints++ = points[i].Y;
		*ptrPoints = points[i].Z;
	}
	pointsMat = pointsMat*(Mat)rotTilt;

	for (int i = 0; i < numPoints; i++)
	{
		points[i].X = (ptrP + (i*step))[0];
		points[i].Y = (ptrP + (i*step))[1];
		points[i].Z = (ptrP + (i*step))[2];
	}
}


void KinectSensor::transformArray(XnPoint3D* points, int numPoints)
{
	if (idCam != idRefCam)
	{
		//Create extended matrices
		Mat pointsMat = Mat(numPoints, 3, CV_32F);
		float* ptrP = (float*)pointsMat.data;
		int stepP = pointsMat.step/sizeof(float);

		Mat translationExt = Mat(numPoints, 3, CV_32F);
		float* ptrT = (float*)translationExt.data;
		int stepT = translationExt.step/sizeof(float);

		float tx = translation(0);
		float ty = translation(1);
		float tz = translation(2);
		for (int i = 0; i < numPoints; i++)
		{ 
			//create points matrix
			float* ptrPoints = ptrP + (i*stepP);
			*ptrPoints++ = points[i].X;
			*ptrPoints++ = points[i].Y;
			*ptrPoints = points[i].Z;

			//create translation matrix
			float* ptrTrans = ptrT + (i*stepT);
			*ptrTrans++ = tx;
			*ptrTrans++ = ty;
			*ptrTrans = tz;
		}
		
		pointsMat = ((Mat)rotation*pointsMat.t()+translationExt.t()).t();
		if (tiltAngle != 0)
			pointsMat = pointsMat*(Mat)rotTilt;

		for (int i = 0; i < numPoints; i++)
		{
			points[i].X = (ptrP + (i*stepP))[0];
			points[i].Y = (ptrP + (i*stepP))[1];
			points[i].Z = (ptrP + (i*stepP))[2];
		}

	}
	else if(tiltAngle != 0)
	{
		//Create extended matrices
		Mat pointsMat = Mat(numPoints, 3, CV_32F);
		float* ptrP = (float*)pointsMat.data;
		int step = pointsMat.step/sizeof(float);

		for (int i = 0; i < numPoints; i++)
		{ 
			//create points matrix
			float* ptrPoints = ptrP + (i*step);
			*ptrPoints++ = points[i].X;
			*ptrPoints++ = points[i].Y;
			*ptrPoints = points[i].Z;
		}
		pointsMat = pointsMat*(Mat)rotTilt;

		for (int i = 0; i < numPoints; i++)
		{
			points[i].X = (ptrP + (i*step))[0];
			points[i].Y = (ptrP + (i*step))[1];
			points[i].Z = (ptrP + (i*step))[2];
		}
	}

}


void KinectSensor::tiltCorrection(XnPoint3D* points, int numPoints)
{
	if(tiltAngle != 0)
	{
		//Create extended matrices
		Mat pointsMat = Mat(numPoints, 3, CV_32F);
		float* ptrP = (float*)pointsMat.data;
		int step = pointsMat.step/sizeof(float);

		for (int i = 0; i < numPoints; i++)
		{ 
			//create points matrix
			float* ptrPoints = ptrP + (i*step);
			*ptrPoints++ = points[i].X;
			*ptrPoints++ = points[i].Y;
			*ptrPoints = points[i].Z;
		}
		pointsMat = pointsMat*(Mat)rotTilt;

		for (int i = 0; i < numPoints; i++)
		{
			points[i].X = (ptrP + (i*step))[0];
			points[i].Y = (ptrP + (i*step))[1];
			points[i].Z = (ptrP + (i*step))[2];
		}
	}

}


Matx31d KinectSensor::transformPoint(const Matx31d& point3D) const
{
	Mat tmp = Mat(rotation)*Mat(point3D);
	Mat out = tmp+Mat(translation);

	return out;
}

	
XnUInt64 KinectSensor::getFocalLength()const
{
	return focalLength;
}
	
XnDouble KinectSensor::getPsize() const
{
	return pSize;
}

	
float KinectSensor::getOx() const
{
	return ox;
}
	
float KinectSensor::getOy() const
{
	return oy;
}

void KinectSensor::initExtrinsics(int id, int idRefCam)
{
	if (id == idRefCam) //reference camera
	{
		rotation(0,0) = 1;
		rotation(1,1) = 1;
		rotation(2,2) = 1;
	}
	else //load from disk
	{
		char fromStr[10], toStr[10];
		itoa(id, fromStr, 10);
		itoa(idRefCam, toStr, 10);

		char *path = "D:\\CameraCalibrations\\extrinsics\\";

		//create filename
		char fileNameRot[150], fileNameTrans[150];
		char fileNameRotInv[150], fileNameTransInv[150];
		char common[50];
		char commonInv[50];
		strcpy(common, fromStr);
		strcat(common, toStr);
		strcat(common, ".xml");

		strcpy(commonInv, toStr);
		strcat(commonInv, fromStr);
		strcat(commonInv, ".xml");


		strcpy(fileNameRot, path);
		strcpy(fileNameTrans, path);
		strcpy(fileNameRotInv, path);
		strcpy(fileNameTransInv, path);

		strcat(fileNameRot, "rotation_");
		strcat(fileNameTrans, "translation_");
		strcat(fileNameRot, common);
		strcat(fileNameTrans, common);

		strcat(fileNameRotInv, "rotation_");
		strcat(fileNameTransInv, "translation_");
		strcat(fileNameRotInv, commonInv);
		strcat(fileNameTransInv, commonInv);

		FileStorage fsRot(fileNameRot, FileStorage::READ);
		FileStorage fsTra(fileNameTrans, FileStorage::READ);
		FileStorage fsRotInv(fileNameRotInv, FileStorage::READ);
		FileStorage fsTraInv(fileNameTransInv, FileStorage::READ);
		if (fsRot.isOpened() && fsTra.isOpened() && fsRotInv.isOpened() && fsTraInv.isOpened())
		{
			Mat t,r, tInv, rInv;
			fsRot["Rotation"] >> r;
			fsTra["Translation"] >> t;
			fsRotInv["Rotation"] >> rInv;
			fsTraInv["Translation"] >> tInv;


			t.copyTo(translation);
			r.copyTo(rotation);
			tInv.copyTo(translationInv);
			rInv.copyTo(rotationInv);

			fsRot.release();
			fsTra.release();
			fsRotInv.release();
			fsTraInv.release();
		}
	}
}

const XnRGB24Pixel* KinectSensor::getRGBMap()
{
	return rgbNode.GetRGB24ImageMap();
}

const XnDepthPixel* KinectSensor::getDepthMap()
{
	return depthNode.GetDepthMap();
}

void KinectSensor::waitAndUpdate()
{
	//context.WaitAndUpdateAll();
	context.WaitAndUpdateAll();
}

void KinectSensor::shutDown()
{
	context.Shutdown();
}

bool KinectSensor::tilt(int angle)
{
	if (angle != 0)
	{
		float rad;
		float b = 0.0094;
		//float b = 0.08;
		float m = 0.0536;
		rad = angle*b + m;
		
		Mat rotTiltM = Mat(3,3, CV_32F); 
		Mat tiltM = Mat(3,1, CV_32F);
		tiltM.at<float>(0) = rad;
		tiltM.at<float>(1) = 0;
		tiltM.at<float>(2) = 0;
		Rodrigues(tiltM, rotTiltM);
		rotTilt = Matx33f(rotTiltM);	


		//And in the opposite direction
		float radNeg = -angle*b + m;
		tiltM.at<float>(0) = radNeg;
		Rodrigues(tiltM, rotTiltM);
		rotTiltNeg = Matx33f(rotTiltM);	

	}


	XnStatus res; 
	tiltAngle = angle;
	if (!m_isOpen)
		open();

	// Send move control request 
	res = xnUSBSendControl(m_dev, XN_USB_CONTROL_TYPE_VENDOR, 0x31, 
		angle, 0x00, NULL, 0, 0); 
	if (res != XN_STATUS_OK) 
	{ 
		xnPrintError(res, "xnUSBSendControl failed"); 
		return false; 
	} 	
	return true; 
}

//private
void KinectSensor::open()
{
	const XnUSBConnectionString *paths; 
	XnUInt32 count; 
	XnStatus res; 

	// Init OpenNI USB 
	res = xnUSBInit(); 
	if (res != XN_STATUS_OK) { 
		xnPrintError(res, "xnUSBInit failed"); 
	} 

	// Open "Kinect motor" USB device 
	res = xnUSBEnumerateDevices(0x045E /* VendorID */, 0x02B0 /*ProductID 
															  */, &paths, &count); 
	if (res != XN_STATUS_OK) { 
		xnPrintError(res, "xnUSBEnumerateDevices failed");  
	} 

	// Open first found device 
//	int ref;
//	if (idCam == 0)
//		ref = 1;
//	else if (idCam == 1)
//		ref = 0;
//	else
//		ref = idCam;
	res = xnUSBOpenDeviceByPath(paths[idCam], &m_dev); 
	if (res != XN_STATUS_OK) { 
		xnPrintError(res, "xnUSBOpenDeviceByPath failed"); 
	} 

	XnUChar buf[1]; // output buffer 

	// Init motor 
	res = xnUSBSendControl(m_dev, (XnUSBControlType) 0xc0, 0x10, 0x00, 
		0x00, buf, sizeof(buf), 0); 
	if (res != XN_STATUS_OK) { 
		xnPrintError(res, "xnUSBSendControl failed"); 
		close(); 
	} 

	res = xnUSBSendControl(m_dev, 
		XnUSBControlType::XN_USB_CONTROL_TYPE_VENDOR, 0x06, 0x01, 0x00, NULL, 
		0, 0); 
	if (res != XN_STATUS_OK) { 
		xnPrintError(res, "xnUSBSendControl failed"); 
		close(); 
	} 
	m_isOpen = true;
}

void KinectSensor::close() 
 { 
	 if (m_isOpen) 
	 { 
		 xnUSBCloseDevice(m_dev); 
		 m_isOpen = false; 
	 } 

} 


void KinectSensor::BuildDepthToGreyMapping(unsigned short *pMapping)
{
	for( int i=0; i<MAX_DEPTH; i++) // for visualisation
		pMapping[i] = 255.0*powf(1.0-((float)i/MAX_DEPTH),3.0); // entirely arbitrary
}
	
void KinectSensor::getDepthImage(Mat& dImage)
{
	const XnDepthPixel* dMap = depthNode.GetDepthMap();
//	Mat* dImage = new Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
	uchar *imagePtr = (uchar*)dImage.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		int charVal = Mapping[dMap[y]];
		imagePtr[3*y]   = charVal;
		imagePtr[3*y+1] = charVal;
		imagePtr[3*y+2] = charVal;
	}

//	return dImage;
}

void KinectSensor::getDepthImage(Mat& dImage, const XnDepthPixel* dMap)
{
	uchar *imagePtr = (uchar*)dImage.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		int charVal = Mapping[dMap[y]];
		imagePtr[3*y]   = charVal;
		imagePtr[3*y+1] = charVal;
		imagePtr[3*y+2] = charVal;
	}
}


void KinectSensor::getRGBImage(Mat& rgbImage, const XnRGB24Pixel* rgbMap)
{
	uchar *imagePtr = (uchar*)rgbImage.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		imagePtr[3*y]   = rgbMap->nBlue;
		imagePtr[3*y+1] = rgbMap->nGreen;
		imagePtr[3*y+2] = rgbMap->nRed;
		rgbMap++;
	}
//	return rgbImage;
}

void KinectSensor::getRGBImage(Mat& rgbImage)
{
//	Mat* rgbImage = new Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
	const XnRGB24Pixel* rgbMap = rgbNode.GetRGB24ImageMap();
	uchar *imagePtr = (uchar*)rgbImage.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		imagePtr[3*y]   = rgbMap->nBlue;
		imagePtr[3*y+1] = rgbMap->nGreen;
		imagePtr[3*y+2] = rgbMap->nRed;
		rgbMap++;
	}
//	return rgbImage;
}


void KinectSensor::createRecorder(Recorder* rec, char* path)
{
	rec->Create(context);
	rec->SetDestination(XN_RECORD_MEDIUM_FILE, path);
	rec->AddNodeToRecording(depthNode, XN_CODEC_NULL);
	rec->AddNodeToRecording(rgbNode, XN_CODEC_NULL);
}

void KinectSensor::releaseRecorder(Recorder* rec)
{
	rec->RemoveNodeFromRecording(depthNode);
	rec->Release();
}
