#include "KinectSensor.h"


KinectSensor::KinectSensor(void)
{
	m_isOpen = false;
	BuildDepthToGreyMapping(Mapping);
	tiltAngle = 0;
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
	
	// Initialise Camera with Extrinsics
	initExtrinsics(id, idRefC);
}
	
void KinectSensor::startDevice()
{
	context.StartGeneratingAll();
}
	
void KinectSensor::stopDevice()
{
	context.StopGeneratingAll();
}

XnPoint3D* KinectSensor::arrayBackProject(const XnPoint3D* depthPoints, int numPoints) const
{
	XnPoint3D* realPoints = new XnPoint3D[numPoints];
	depthNode.ConvertProjectiveToRealWorld(numPoints, depthPoints, realPoints);
	return realPoints;
}

Point KinectSensor::pointProject(const Matx31d& point3D) const
{
	//XnPoint3D realPoint;
	//realPoint.X = point3D(0); realPoint.Y = point3D(1); realPoint.Z = point3D(2);
	//XnPoint3D camPoint;
	//depthNode.ConvertRealWorldToProjective(1, &realPoint, &camPoint);
	//Point p;
	//p.x = camPoint.X;
	//p.y = camPoint.Y;
	//return p;


	Point P;
	P.x = (int) (ox + (point3D.val[0]/point3D.val[2])*(focalLength/pSize));
	P.y = (int) (oy + (point3D.val[1]/point3D.val[2])*(focalLength/pSize));
	return P;
}
	
Matx31d KinectSensor::pointBackproject(const Matx31d& point2D) const
{
	//XnPoint3D realPoint;
	//XnPoint3D camPoint;
	//camPoint.X = point2D(0);
	//camPoint.Y = point2D(1);
	//camPoint.Z = point2D(2);

	//depthNode.ConvertProjectiveToRealWorld(1, &camPoint, &realPoint);
	//Matx31d out (realPoint.X, realPoint.Y, realPoint.Z);
	//return out;


	Matx31d p3d;
	p3d(0) = (point2D(0) - ox)*pSize*point2D(2)/focalLength;
	p3d(1) = (point2D(1) - oy)*pSize*point2D(2)/focalLength;
	p3d(2) = point2D(2);
	return p3d;
}


//Transform the array points into a common coordinate system defined by idRefCam
void KinectSensor::transformArray(XnPoint3D* points, int numPoints)
{
	Matx33f rotTilt;
	if (tiltAngle != 0)
	{
		float rad = CV_PI*(tiltAngle)/180;
		Mat rotTiltM = Mat(3,3, CV_32F); 
		Mat tiltM = Mat(3,1, CV_32F);
		tiltM.at<float>(0) = -rad;
		tiltM.at<float>(1) = 0;
		tiltM.at<float>(2) = 0;
		Rodrigues(tiltM, rotTiltM);
		rotTilt = Matx33f(rotTiltM);
	}

	if (idCam != idRefCam)
	{
		for (int i = 0; i < numPoints; i++)
		{
			Matx31f p(points[i].X, points[i].Y, points[i].Z);

			if (tiltAngle != 0)
				p = rotTilt*p;

			Matx31f out = rotation*p+translation;

			points[i].X = out(0);
			points[i].Y = out(1);
			points[i].Z = out(2);
		}
	}
	//Recover tilt angle in the reference CS
	else if(tiltAngle != 0)
	{
		for (int i = 0; i < numPoints; i++)
		{
			Matx31f p(points[i].X, points[i].Y, points[i].Z);
			Matx31f out = rotTilt*p;

			points[i].X = out(0);
			points[i].Y = out(1);
			points[i].Z = out(2);
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
		char common[50];
		strcpy(common, fromStr);
		strcat(common, toStr);
		strcat(common, ".xml");

		strcpy(fileNameRot, path);
		strcpy(fileNameTrans, path);
		strcat(fileNameRot, "rotation_");
		strcat(fileNameTrans, "translation_");
		strcat(fileNameRot, common);
		strcat(fileNameTrans, common);

		FileStorage fsRot(fileNameRot, FileStorage::READ);
		FileStorage fsTra(fileNameTrans, FileStorage::READ);
		if (fsRot.isOpened() && fsTra.isOpened())
		{
			Mat t,r;
			fsRot["Rotation"] >> r;
			fsTra["Translation"] >> t;

			translation = t;
			rotation = r;
			fsRot.release();
			fsTra.release();
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
	context.WaitAndUpdateAll();
}

void KinectSensor::shutDown()
{
	context.Shutdown();
}

bool KinectSensor::tilt(int angle)
{
	XnStatus res; 
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
	tiltAngle = angle;
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
