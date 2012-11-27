#include "ForegroundObjs.h"


ForegroundObjs::ForegroundObjs(void)
{
	numObjects = 0;
}


ForegroundObjs::ForegroundObjs(bool t)
{
	trans = t;
	numObjects = 0;
}


ForegroundObjs::~ForegroundObjs(void)
{
}

void ForegroundObjs::recoverFPoints()
{
	fp2D = new XnPoint3D*[numObjects];
	fp3D = new XnPoint3D*[numObjects];
	numFp = new int[numObjects];
	peopleDistr = new distribution[numObjects];

	for (int i = 0; i < numObjects; i++)
	{
		Rect bb = bboxes[i];
		assert(bb.area() == (bb.height*bb.width));
		fp2D[i] = new XnPoint3D[bb.area()]; //TODO: check 

		int cont = 0;
		for (int y = bb.tl().y; y < bb.br().y; y++)
		{
			for (int x = bb.tl().x; x < bb.br().x; x++)
			{
				ushort depth = fDepth.ptr<ushort>(y)[x];
				if (depth != 0)
				{
					fp2D[i][cont].X = x;
					fp2D[i][cont].Y = y;
					fp2D[i][cont].Z = depth;
					cont++;
				}
			}
		}
		numFp[i] = cont;
		//transform 2D to 3D;
		fp3D[i] = kinect->arrayBackProject(fp2D[i], numFp[i]);
		if (trans)
			kinect->transformArray(fp3D[i], numFp[i]);
	}
	calculateDistrParam();
}

void ForegroundObjs::calculateDistrParam()
{
	for (int i = 0; i < numObjects; i++)
	{
		double sumX, sumY, sumZ, sumXX, sumZZ, sumXZ;
		sumX = sumY = sumZ = sumXX = sumZZ = sumXZ = 0.0;
		int sumW = numFp[i];
		for (int p = 0; p < sumW; p++)
		{
			float x = fp3D[i][p].X;
			float y = fp3D[i][p].Y;
			float z = fp3D[i][p].Z;

			sumX += x;
			sumY += y;
			sumZ += z;

			sumXX += x*x;
			sumZZ += z*z;
			sumXZ += x*z;

		}
		XnPoint3D mean;
		mean.X = sumX/numFp[i];
		mean.Y = sumY/numFp[i];
		mean.Z = sumZ/numFp[i];

		Mat covMat(2,2, CV_64F);
		double covXZ = (sumXZ/sumW)-(mean.X*mean.Z);
		covMat.at<double>(0,0) = (sumXX/sumW)-powf(mean.X,2);
		covMat.at<double>(0,1) = covXZ;
		covMat.at<double>(1,0) = covXZ;
		covMat.at<double>(1,1) = (sumZZ/sumW)-powf(mean.Z,2);
		
		peopleDistr[i].mean = mean;
		peopleDistr[i].cov = covMat;
	}
}

void ForegroundObjs::initialize()
{
	for (int p = 0; p < numObjects; p++)
	{
		delete[] fp2D[p];
		delete[] fp3D[p];
	}
	delete[] fp2D;
	delete[] fp3D;
	delete[] numFp;
//	delete &fDepth;
	delete[] bboxes;
	numObjects = 0;
}

void ForegroundObjs::setBBoxes(Rect* bb)
{
	bboxes = new Rect[numObjects];
	for (int i = 0; i < numObjects; i++)
	{
		bboxes[i] = bb[i];
	}
}
