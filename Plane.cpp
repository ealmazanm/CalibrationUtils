#include "Plane.h"


Plane::Plane(void)
{
}


Plane::~Plane(void)
{
}

ofstream outDebug1("D:\\debug.txt", ios::out);

//Implementation of public methods

int	Plane::frontoPlaneFit(const XnDepthPixel* Depths, const KinectSensor* kinect, Rect window, float minDepth, float maxDepth)
{
	int NumberOfPoints=0,iMinNew=XN_VGA_Y_RES,jMinNew=XN_VGA_X_RES,iMaxNew=0,jMaxNew=0;
	int iMin=window.y,jMin=window.x,iMax=iMin+window.height,jMax=jMin+window.width;
	double x,y,z,w,e,en,wx,wy,sigmaX,sigmaY;
	double sumXX=0.0,sumXY=0.0,sumXZ=0.0,sumYY=0.0,sumYZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0,sumW=0.0;
	
	// Local origin (represent in other coordiante system to reduce the error.)
	int j0=(int)(0.5*(jMax+jMin)), i0=(int)(0.5*(iMax+iMin));
	for(int j=j0;j<jMax;j++) if(Depths[i0*XN_VGA_X_RES+j]!=0) { j0=j; break;} //why this local cs?
	
	double z0 = (double) Depths[i0*XN_VGA_X_RES+j0];
	Matx31d point2D(j0, i0, z0);
	Matx31d localOrigin = kinect->pointBackproject(point2D);

	// INITIAL FIT
	for (int i=iMin; i<iMax; i++)
	{
		for (int j=jMin; j<jMax; j++)
		{
			// Recover 3D point
			int depth = Depths[i*XN_VGA_X_RES+j];
			w = 1; // always 1?
			if((depth!=0)&&(depth>minDepth)&&(depth<maxDepth))
			{
				point2D = Matx31d(j,i,depth);
				Matx31d point3d = kinect->pointBackproject(point2D);
				x = point3d(0)-localOrigin(0);  
				y = point3d(1)-localOrigin(1);
				z = point3d(2)-localOrigin(2);

				wx = w*x;
				wy = w*y;

				sumXX += wx*x;
				sumXY += wx*y;
				sumYY += wy*y;
				sumXZ += wx*z;
				sumYZ += wy*z;
				sumX  += wx;
				sumY  += wy;
				sumZ  += w*z;
				sumW  += w;

				++NumberOfPoints;
			}
		}
	}

	if(sumW==0.0) return -1;

	Matx33d LeftMatrix(sumXX,sumXY,sumX,sumXY,sumYY,sumY,sumX,sumY,sumW);
	Matx31d RightVector(sumXZ,sumYZ,sumZ);
	Matx33d Inverse = LeftMatrix.inv();
	parameters = Inverse * RightVector;

	centroid.val[0]=sumX/sumW;
	centroid.val[1]=sumY/sumW;
	centroid.val[2]=sumZ/sumW;
	centroid += localOrigin; //back to camera coordiante system

	residualVariance = 20*20; // 3cm variance - function of z0 in the future? 

	// Resize window
	sigmaX = powf(sumXX/sumW-powf(sumX/sumW,2.0),0.5); //simga??
	sigmaY = powf(sumYY/sumW-powf(sumY/sumW,2.0),0.5);
	Point corner1 = kinect->pointProject(centroid-Matx31d(sigmaX,sigmaY,0.0)*2.0);
	Point corner2 = kinect->pointProject(centroid+Matx31d(sigmaX,sigmaY,0.0)*2.0);
	iMin=corner1.y; jMin=corner1.x; iMax=corner2.y; jMax=corner2.x;
	iMin=max(0,iMin); iMax=min(iMax,XN_VGA_Y_RES);
	jMin=max(0,jMin); jMax=min(jMax,XN_VGA_X_RES);

	// ITERATIVE FIT
	int numIteration=0,MaxIterations=3;

	while (numIteration++<MaxIterations)
	{
		NumberOfPoints = 0;
		sumXX=0.0,sumXY=0.0,sumXZ=0.0,sumYY=0.0,sumYZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0,sumW=0.0;
		
		for (int i=iMin; i<iMax; i++)
		{
			for (int j=jMin; j<jMax; j++)
			{
				// Recover 3D point
				int depth = Depths[i*XN_VGA_X_RES+j];

				if(depth!=0)
				{
					point2D = Matx31d(j,i,depth);
					Matx31d point3d = kinect->pointBackproject(point2D);
					x = point3d(0)-localOrigin(0);  
					y = point3d(1)-localOrigin(1);
					z = point3d(2)-localOrigin(2);

					e = z-(parameters.val[0]*x+parameters.val[1]*y+parameters.val[2]); //error
					en = e/sqrt(residualVariance); //error normalized?
					w = TukeyBiweight(en,3.0);

					if(w>0.0) //only the points placed whithin the residualVariance area.
					{
						//					if(i<iMinNew) iMinNew=i;
						//					if(i>iMaxNew) iMaxNew=i;
						//					if(j<jMinNew) jMinNew=j;
						//					if(j>jMaxNew) jMaxNew=j;

						wx = w*x;
						wy = w*y;

						sumXX += wx*x;
						sumXY += wx*y;
						sumYY += wy*y;
						sumXZ += wx*z;
						sumYZ += wy*z;
						sumX  += wx;
						sumY  += wy;
						sumZ  += w*z;
						sumW  += w;

						++NumberOfPoints;
					}
				}
			}
		}

		if(sumW==0.0) return -1;

		LeftMatrix = Matx33d(sumXX,sumXY,sumX,sumXY,sumYY,sumY,sumX,sumY,sumW);
		RightVector = Matx31d(sumXZ,sumYZ,sumZ);
		Inverse = LeftMatrix.inv();
		parameters = Inverse * RightVector;

		centroid.val[0]=sumX/sumW;
		centroid.val[1]=sumY/sumW;
		centroid.val[2]=sumZ/sumW;
		centroid += localOrigin; //back to Kinect coordiante system

		// Resize window
		sigmaX = powf(sumXX/sumW-powf(sumX/sumW,2.0),0.5); //simga??
		sigmaY = powf(sumYY/sumW-powf(sumY/sumW,2.0),0.5);
		corner1 = kinect->pointProject(centroid-Matx31d(sigmaX,sigmaY,0.0)*2.0);
		corner2 = kinect->pointProject(centroid+Matx31d(sigmaX,sigmaY,0.0)*2.0);
		iMin=corner1.y; jMin=corner1.x; iMax=corner2.y; jMax=corner2.x;
		iMin=max(0,iMin); iMax=min(iMax,XN_VGA_Y_RES);
		jMin=max(0,jMin); jMax=min(jMax,XN_VGA_X_RES);

		fitWindow.y=iMin; fitWindow.height = iMax-iMin;
		fitWindow.x=jMin; fitWindow.width  = jMax-jMin;

	}

	// move origin back to kinect coordinate system
	parameters.val[2] -= (parameters.val[0]*localOrigin(0)+parameters.val[1]*localOrigin(1)-z0);

	double norm=sqrt(parameters.val[0]*parameters.val[0]+parameters.val[1]*parameters.val[1]+1.0);
	normal.val[0] = parameters.val[0]/norm;
	normal.val[1] = parameters.val[1]/norm;
	normal.val[2] = -1.0/norm;

	return 0;
}
	
int	Plane::updatePlaneFit(const XnDepthPixel* Depths, KinectSensor* kinect, Mat& weighMat, const XnRGB24Pixel* colors)
{
	int NumberOfPoints=0;
	int NumberOfDistPoints = 0;
	int iMin=fitWindow.y,jMin=fitWindow.x,iMax=iMin+fitWindow.height,jMax=jMin+fitWindow.width;
	double x,y,z,w,e,en,we,wx,wy, wz,sigmaX,sigmaY, sigmaZ;
	double sumXX=0.0,sumXY=0.0,sumXZ=0.0,sumYY=0.0,sumYZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0,sumW=0.0,sumEE=0.0, sumZZ=0.0;

	// Local origin
	Matx31d localOrigin(centroid(0), centroid(1), centroid(2));

	parameters.val[2] += (parameters.val[0]*localOrigin(0)+parameters.val[1]*localOrigin(1)-localOrigin(2)); //C represented wrt the local cs

	// ITERATIVE FIT
	int numIteration=0,MaxIterations=3;

	float maxX, minX, maxY, minY;
	maxX = minX = maxY = minY = 0.0;
	bool init = false;
	double std = 0.0;
	

	while (numIteration++<MaxIterations)
	{
		double sumColorDistWeight = 0.0, sumWeight=0.0;

		Point centr2D = kinect->pointProject(centroid);
		XnRGB24Pixel centrColor = colors[centr2D.y*XN_VGA_X_RES+centr2D.x];
		Vec3b centrColorHSV = Utils::RGBtoHSV(centrColor.nRed, centrColor.nGreen, centrColor.nBlue);

		NumberOfPoints = 0;
		NumberOfDistPoints=0;
		sumXX=0.0,sumXY=0.0,sumXZ=0.0,sumYY=0.0,sumYZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0,sumW=0.0, sumZZ = 0.0;
		
		for (int i=iMin; i<iMax; i++)
		{
			float* weithPtr = weighMat.ptr<float>(i);
			for (int j=jMin; j<jMax; j++)
			{
				// Recover 3D point
				int depth = Depths[i*XN_VGA_X_RES+j];

				if(depth!=0)
				{
					XnRGB24Pixel pointColor = colors[i*XN_VGA_X_RES+j];
					Vec3b pointColorHSV = Utils::RGBtoHSV(pointColor.nRed, pointColor.nGreen, pointColor.nBlue);
					float colorDist = powf(centrColorHSV[0]-pointColorHSV[0],2) + powf(centrColorHSV[1]-pointColorHSV[1],2);
					double wColor = 1;
					if (numIteration > 1)
					{	
						double colorDistN = colorDist/(3*std); 
						wColor = TukeyBiweight(colorDistN, 3.0);
//						weithPtr[j] = wDist;
					}
					if (wColor > 0.0)
					{
						Matx31d point2D = Matx31d(j,i,depth);
						Matx31d point3d = kinect->pointBackproject(point2D);
						x = point3d(0)-localOrigin(0);  
						y = point3d(1)-localOrigin(1);
						z = point3d(2)-localOrigin(2);
						
						e = z-(parameters.val[0]*x+parameters.val[1]*y+parameters.val[2]);
						en = e/sqrt(residualVariance);
						w = TukeyBiweight(en,3.0);
						weithPtr[j] = wColor;
						if(w>0.0)
						{
							sumColorDistWeight += (wColor*colorDist);
							sumWeight += wColor;
//							weithPtr[j] = w;
					
							wx = w*x;
							wy = w*y;
							wz = w*z;
							we = w*e;

							sumXX += wx*x;
							sumXY += wx*y;
							sumYY += wy*y;
							sumXZ += wx*z;
							sumYZ += wy*z;
							sumZZ += wz*z;
							sumEE += we*e;
							sumX  += wx;
							sumY  += wy;
							sumZ  += w*z;
							sumW  += w;

							++NumberOfPoints;
						}
						else weithPtr[j]=0.0;
					}
					else weithPtr[j]=0.0;
				}
				else weithPtr[j]=0.0;
			}
		}

		if(sumW==0.0) 
			return -1;
		

		Matx33d LeftMatrix = Matx33d(sumXX,sumXY,sumX,sumXY,sumYY,sumY,sumX,sumY,sumW);
		Matx31d RightVector = Matx31d(sumXZ,sumYZ,sumZ);
		Matx33d Inverse = LeftMatrix.inv();
		parameters = Inverse * RightVector;

//		residualVariance = sumEE/sumW; // Proper way to calculate residual variance but template around plane too small when plane is moving fast
		residualVariance = 20*20; // Wrong way! 3cm variance - function of z0 in the future? 

		centroid.val[0]=sumX/sumW;
		centroid.val[1]=sumY/sumW;
		centroid.val[2]=sumZ/sumW;
		
		// Resize window
		sigmaX = powf(sumXX/sumW-powf(sumX/sumW,2.0),0.5); // std in x
		sigmaY = powf(sumYY/sumW-powf(sumY/sumW,2.0),0.5); // std in y
		sigmaZ = powf(sumZZ/sumW-powf(sumZ/sumW,2.0),0.5); // std in z

		normal.val[0] = parameters.val[0];
		normal.val[1] = parameters.val[1];
		normal.val[2] = -1.0;

		Matx31d yOrt(0,1,0);
		Matx31d xOrt(1,0,0);		

		centroid +=localOrigin;

		Matx31d orthogVecXZ = Mat(normal).cross(Mat(yOrt));
		Matx31d orthogVecYZ = Mat(normal).cross(Mat(xOrt));

			//normalize vectors
		double normXZ=sqrt(powf(orthogVecXZ(0),2)+ powf(orthogVecXZ(1),2) + powf(orthogVecXZ(2),2));
		orthogVecXZ(0) = orthogVecXZ(0)/normXZ;
		orthogVecXZ(1) = orthogVecXZ(1)/normXZ;
		orthogVecXZ(2) = orthogVecXZ(2)/normXZ;

		double normYZ=sqrt(powf(orthogVecYZ(0),2)+ powf(orthogVecYZ(1),2) + powf(orthogVecYZ(2),2));
		orthogVecYZ(0) /= normYZ;
		orthogVecYZ(1) /= normYZ;
		orthogVecYZ(2) /= normYZ;

		//Factor to multiply the unit vector (sigma X/Y and sigma Z)
		double multFactX = sigmaX+(abs(sigmaZ*orthogVecXZ(2))); //abs in order to avoid sign changing if orthog < 0
		double multFactY = sigmaY+(abs(sigmaZ*orthogVecYZ(2)));

		//increase direction vector
		orthogVecXZ(0) *= multFactX;
		orthogVecXZ(1) *= multFactX;
		orthogVecXZ(2) *= multFactX;
		orthogVecYZ(0) *= multFactY;
		orthogVecYZ(1) *= multFactY;
		orthogVecYZ(2) *= multFactY;

	
		Matx31f p1Y = centroid+Matx31d(0, orthogVecYZ(1), orthogVecYZ(2))*2;
		Matx31f p2Y = centroid-Matx31d(0, orthogVecYZ(1), orthogVecYZ(2))*2;

		Matx31f p1X = centroid+Matx31d(orthogVecXZ(0), 0, orthogVecXZ(2))*2;
		Matx31f p2X = centroid-Matx31d(orthogVecXZ(0), 0, orthogVecXZ(2))*2;

		Point c1X = kinect->pointProject(p1X);
		Point c2X = kinect->pointProject(p2X);

		Point c1Y = kinect->pointProject(p1Y);
		Point c2Y = kinect->pointProject(p2Y);

		Point corner1, corner2;
		corner2.x = c1X.x; corner1.y = c1Y.y;
		corner1.x = c2X.x; corner2.y = c2Y.y;
		
		iMin=corner1.y; jMin=corner1.x; iMax=corner2.y; jMax=corner2.x;
		iMin=max(0,iMin); iMax=min(iMax,XN_VGA_Y_RES);
		jMin=max(0,jMin); jMax=min(jMax,XN_VGA_X_RES);
		
		fitWindow.y=iMin; fitWindow.height = iMax-iMin;
		fitWindow.x=jMin; fitWindow.width  = jMax-jMin;
		
		std = (sumColorDistWeight/sumWeight)/1.5;
	}
	// move origin back to kinect coordinate system
	parameters.val[2] -= (parameters.val[0]*localOrigin(0)+parameters.val[1]*localOrigin(1)-localOrigin(2));

	double norm=sqrt(parameters.val[0]*parameters.val[0]+parameters.val[1]*parameters.val[1]+1.0);
	normal.val[0] = parameters.val[0]/norm;
	normal.val[1] = parameters.val[1]/norm;
	normal.val[2] = -1.0/norm;

	return 0;
}


//getters and setters
const Matx31d* Plane::getParameters() const
{
	return &parameters;
}

	
const Matx31d* Plane::getCentroid() const
{
	return &centroid;
}
	
const Matx31d* Plane::getNormal() const
{
	return &normal;
}

	
void Plane::setParameters(Matx31d* p)
{
	parameters = *p;
}
	
void Plane::setCentroid(Matx31d* c)
{
	centroid = *c;
}
	
void Plane::setNormal(Matx31d* n)
{
	normal = *n;
}

double Plane::TukeyBiweight(double e, double c) const 
{
	if(fabs(e)<c)
		return(powf(1.0-powf(e/3.0,2.0),2.0)); //?
	else
		return(0.0);
}

Rect Plane::getFitWindow() const
{
	return fitWindow;
}

void Plane::setResidualVariance(double rv)
{
	residualVariance = rv;
}

double Plane::getResidualVariance() const
{
	return residualVariance;
}

double Plane::getDistance() const
{

	return -parameters(2)/(parameters(0)*normal(0) + parameters(1)*normal(1) - normal(2));
}

void Plane::setFittingWindow(Point p, int width, int height)
{
	fitWindow.x = p.x;
	fitWindow.y = p.y;
	fitWindow.width = width;
	fitWindow.height = height;
}