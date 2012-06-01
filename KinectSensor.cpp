#include "KinectSensor.h"


KinectSensor::KinectSensor(void)
{
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
		while (contNodes < idCam)
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
	Point P;
	P.x = (int) (ox + (point3D.val[0]/point3D.val[2])*(focalLength/pSize));
	P.y = (int) (oy + (point3D.val[1]/point3D.val[2])*(focalLength/pSize));
	return P;
}
	
Matx31d KinectSensor::pointBackproject(const Matx31d& point2D) const
{
	Matx31d p3d;
	p3d(0) = (point2D(0) - ox)*pSize*point2D(2)/focalLength;
	p3d(1) = (point2D(1) - oy)*pSize*point2D(2)/focalLength;
	p3d(2) = point2D(2);
	return p3d;
}

void KinectSensor::transformArray(XnPoint3D* points, int numPoints) const
{
	for (int i = 0; i < numPoints; i++)
	{
		Matx31f p(points[i].X, points[i].Y, points[i].Z);
		Matx31f out = rotation*p+translation;

		points[i].X = out(0);
		points[i].Y = out(1);
		points[i].Z = out(2);
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
		strcat(common, ".yml");

		strcpy(fileNameRot, path);
		strcpy(fileNameTrans, path);
		strcat(fileNameRot, "rotation_");
		strcat(fileNameTrans, "translation_");
		strcat(fileNameRot, common);
		strcat(fileNameTrans, common);

		FileStorage fsRot(fileNameRot, FileStorage::READ);
		FileStorage fsTra(fileNameTrans, FileStorage::READ);

		Mat t,r;
		fsRot["Rotation"] >> r;
		fsTra["Translation"] >> t;

		translation = t;
		rotation = r;
		fsRot.release();
		fsTra.release();
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