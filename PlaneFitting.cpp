#include "PlaneFitting.h"



int PlaneFitting::NO_TRACKING = 0;
int PlaneFitting::BUILD_MODEL = 1;
int PlaneFitting::TRACK_MODEL = 2;
int PlaneFittingCOUNTDOWN_MODEL = 3;

void PlaneFitting::recoverPlanes(KinectSensor& kinect, KinectSensor& kinectRef, const XnDepthPixel* depthMap, const XnDepthPixel* depthMapRef, const XnRGB24Pixel* rgbMap, const XnRGB24Pixel* rgbMapRef, Rect& selectWindow, Plane* plane, Plane* planeRef, int& trackingMode, int& numPlanes)
{
	int err, errRef;
	if( trackingMode!=NO_TRACKING )
	{
		// Build model
		if( trackingMode==BUILD_MODEL )
		{
			err = plane->frontoPlaneFit(depthMap, &kinect, selectWindow);
			errRef = planeRef->frontoPlaneFit(depthMapRef, &kinectRef, selectWindow);
			if((err!=0)||(plane->getFitWindow().area()<=1)||(errRef!=0)||(planeRef->getFitWindow().area()<=1))
			{
				trackingMode = NO_TRACKING;
				cout << "No initial plane found";
			}
			else trackingMode=TRACK_MODEL;
		}
		// Track Model
		else
		{
			err = plane->updatePlaneFit(depthMap, &kinect, rgbMap);
			errRef = planeRef->updatePlaneFit(depthMapRef, &kinectRef, rgbMapRef);
			
			if((err!=0)||(plane->getFitWindow().area()<=1)||(errRef!=0)||(planeRef->getFitWindow().area()<=1))
			{
				trackingMode=NO_TRACKING;
				plane->setResidualVariance(900);
				planeRef->setResidualVariance(900);
				cout << "Tracked plane lost\n";
				numPlanes -= 3;
				if (numPlanes < 0)
					numPlanes = 0;
				cout << "3 planes were removed. " << endl;
			}
		}
	}
}