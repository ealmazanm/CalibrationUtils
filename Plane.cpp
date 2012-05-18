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
	int iMin=fitWindow.y,jMin=fitWindow.x,iMax=iMin+fitWindow.height,jMax=jMin+fitWindow.width;
	double x,y,z,w,e,en,we,wx,wy,sigmaX,sigmaY;
	double sumXX=0.0,sumXY=0.0,sumXZ=0.0,sumYY=0.0,sumYZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0,sumW=0.0,sumEE=0.0;

	// Local origin
	Matx31d localOrigin(centroid(0), centroid(1), centroid(2));

	parameters.val[2] += (parameters.val[0]*localOrigin(0)+parameters.val[1]*localOrigin(1)-localOrigin(2)); //C represented wrt the local cs

	// ITERATIVE FIT
	int numIteration=0,MaxIterations=3;

	float maxX, minX, maxY, minY;
	maxX = minX = maxY = minY = 0.0;
	bool init = false;

	while (numIteration++<MaxIterations)
	{
		Point centr2D = kinect->pointProject(centroid);
		XnRGB24Pixel centrColor = colors[centr2D.y*XN_VGA_X_RES+centr2D.x];

		NumberOfPoints = 0;
		sumXX=0.0,sumXY=0.0,sumXZ=0.0,sumYY=0.0,sumYZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0,sumW=0.0;
		
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
					double dist = sqrt(powf((int)(centrColor.nBlue-pointColor.nBlue),2) + powf((int)(centrColor.nGreen-pointColor.nGreen),2) + powf((int)(centrColor.nRed-pointColor.nRed),2));
					double distN = dist/35;
					double wDist = TukeyBiweight(distN, 3.0);
					if (wDist > 0.5)
					{

						Matx31d point2D = Matx31d(j,i,depth);
						Matx31d point3d = kinect->pointBackproject(point2D);
						x = point3d(0)-localOrigin(0);  
						y = point3d(1)-localOrigin(1);
						z = point3d(2)-localOrigin(2);

						e = z-(parameters.val[0]*x+parameters.val[1]*y+parameters.val[2]);
						en = e/sqrt(residualVariance);
						w = TukeyBiweight(en,3.0);
						weithPtr[j] = w;
						if(w>0.3)
						{
							wx = w*x;
							wy = w*y;
							we = w*e;

							if (!init)
							{
								minX = maxX = x;
								minY = maxY = y;
								init = true;
							}
							else
							{
								if (x > maxX)
									maxX = x;
								else if (x < minX)
									minX = x;
								if (y > maxY)
									maxY = y;
								else if (y < minY)
									minY = y;
							}


							sumXX += wx*x;
							sumXY += wx*y;
							sumYY += wy*y;
							sumXZ += wx*z;
							sumYZ += wy*z;
							sumEE += we*e;
							sumX  += wx;
							sumY  += wy;
							sumZ  += w*z;
							sumW  += w;

							++NumberOfPoints;
						}
					}
				}
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
		centroid += localOrigin;
		// Resize window
		sigmaX = powf(sumXX/sumW-powf(sumX/sumW,2.0),0.5); //simga??
		sigmaY = powf(sumYY/sumW-powf(sumY/sumW,2.0),0.5);


	/*	Matx31f c1_3d, c2_3d;

		minX -= 10;
		maxX += 10;
		minY -= 10;
		maxY += 10;

		if (abs(minX - centroid(0)) > 300)
			minX = centroid(0) - 300;
		if (abs(maxX - centroid(0)) > 300)
			maxX = centroid(0) + 300;

		if (abs(minY - centroid(1)) > 200)
			minY = centroid(1) - 200;
		if (abs(maxY - centroid(1)) > 200)
			maxY = centroid(1) + 200;*/

		//Matx31f c1 (minX, minY, centroid(2));
		//c1(2) = parameters(0)*c1(0) + parameters(1)*c1(1) +  parameters(2);
		//Matx31f c2 (maxX, maxY, centroid(2));
		//c2(2) = parameters(0)*c2(0) + parameters(1)*c2(1) +  parameters(2);
		//c1 += localOrigin; c2 += localOrigin;
		//Point corner1 = kinect->pointProject(c1);
		//Point corner2 = kinect->pointProject(c2);

	/*	Matx31f tmp1(0, minY, centroid(2));
		Matx31f tmp11(0, maxY, centroid(2));
		tmp1(2) = parameters(0)*tmp1(0) + parameters(1)*tmp1(1) +  parameters(2);
		tmp11(2) = parameters(0)*tmp11(0) + parameters(1)*tmp11(1) +  parameters(2);
		tmp1 +=  localOrigin; tmp11 +=  localOrigin; 
		Point X1 = kinect->pointProject(Matx31f(0, tmp1(1), tmp1(2)));
		Point X11 = kinect->pointProject(Matx31f(0, tmp11(1), tmp11(2)));



		Matx31f tmp2(minX, 0, centroid(2));
		Matx31f tmp22(maxX, 0, centroid(2));
		tmp2(2) = parameters(0)*tmp2(0) + parameters(1)*tmp2(1) +  parameters(2);
		tmp22(2) = parameters(0)*tmp22(0) + parameters(1)*tmp22(1) +  parameters(2);
		tmp2 +=  localOrigin; tmp22 += localOrigin; 
		Point X2 = kinect->pointProject(Matx31f(tmp2(0), 0, tmp2(2)));
		Point X22 = kinect->pointProject(Matx31f(tmp22(0), 0, tmp22(2)));

		Point corner1, corner2;
		corner1.x = X2.x; corner1.y = X1.y;
		corner2.x = X22.x; corner2.y = X11.y;*/


		//Matx31d c1_3d, c2_3d;
		//c1_3d = centroid-Matx31d(sigmaX, sigmaY, 0.0)*2.3;
		//c2_3d = centroid+Matx31d(sigmaX, sigmaY, 0.0)*2.3;
		//c1_3d(2) = parameters(0)*c1_3d(0)+parameters(1)*c1_3d(1) + parameters(2);
		//c2_3d(2) = parameters(0)*c2_3d(0) + parameters(1)*c2_3d(1) + parameters(2);
	
		//c1_3d += localOrigin;
		//c2_3d += localOrigin;

		//Point corner1 = kinect->pointProject(c1_3d);
		//Point corner2 = kinect->pointProject(c2_3d);

	/*	Point corner1 = kinect->pointProject(centroid);
		Point corner2 = kinect->pointProject(centroid);
		corner1.x -= 100; corner1.y -= 70;
		corner2.x +=100; corner2.y += 70;*/

		Point corner1 = kinect->pointProject(centroid-Matx31d(sigmaX,sigmaY,0.0)*2.0);
		Point corner2 = kinect->pointProject(centroid+Matx31d(sigmaX,sigmaY,0.0)*2.0);
		iMin=corner1.y; jMin=corner1.x; iMax=corner2.y; jMax=corner2.x;
		iMin=max(0,iMin); iMax=min(iMax,XN_VGA_Y_RES);
		jMin=max(0,jMin); jMax=min(jMax,XN_VGA_X_RES);
		
		fitWindow.y=iMin; fitWindow.height = iMax-iMin;
		fitWindow.x=jMin; fitWindow.width  = jMax-jMin;

		//centroid += localOrigin;
	}
//	centroid += localOrigin;
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