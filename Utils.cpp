#include "Utils.h"


Vec3b Utils::RGBtoHSV(int r, int g, int b)
{
	//Create output
	Vec3b hsvOut;

	//find min and max RGB values
	int rgbMin = MIN3(r, g, b);
	int rgbMax = MAX3(r, g, b);

	//compute Value ( third component of hsvOut)
	hsvOut[2] = rgbMax;
	if (hsvOut[2] == 0)
	{
		hsvOut[0] = 0;
		hsvOut[1] = 0;
		return hsvOut;
	}

	//Normalize value to 1
	r /= hsvOut[2];
	g /= hsvOut[2];
	b /= hsvOut[2];

	//find min and max RGB values
	rgbMin = MIN3(r, g, b);
	rgbMax = MAX3(r, g, b);

	//compute saturation (Second component of hsvOut)
	hsvOut[1] = rgbMax - rgbMin;
	if (hsvOut[1] == 0) 
	{
		hsvOut[0] = 0;
		return hsvOut;
	}

	/* Normalize saturation to 1 */
	r = (r - rgbMin)/(rgbMax - rgbMin);
	g = (g - rgbMin)/(rgbMax - rgbMin);
	b = (b - rgbMin)/(rgbMax - rgbMin);
	rgbMin = MIN3(r, g, b);
	rgbMax = MAX3(r, g, b);

	// Compute hue  (First component of hsvOut
	if (rgbMax == r) 
	{
		hsvOut[0] = 0.0 + 60.0*(g - b);
		if (hsvOut[0] < 0.0) 
		{
			hsvOut[0] += 360.0;
		}
	} else if (rgbMax == g) 
	{
		hsvOut[0] = 120.0 + 60.0*(b - r);
	} else /* rgb_max == rgb.b */ 
	{
		hsvOut[0] = 240.0 + 60.0*(r - g);
	}

	return hsvOut;
}

void Utils::combineTwoImages(const Mat* Frame1, const Mat* Frame2, Mat& Frame12)
{
	for (int i = 0; i < Frame12.rows; i++)
	{
		uchar* ptr12 = Frame12.ptr<uchar>(i);
		const uchar* ptr1 = Frame1->ptr<uchar>(i);
		const uchar* ptr2 = Frame2->ptr<uchar>(i);
		for (int j = 0; j < Frame12.cols; j++)
		{
			if (j < Frame1->cols)
			{
				ptr12[j*3] = ptr1[j*3];
				ptr12[j*3+1] = ptr1[j*3+1];
				ptr12[j*3+2] = ptr1[j*3+2];
			}
			else
			{
				ptr12[j*3] = ptr2[j*3];
				ptr12[j*3+1] = ptr2[j*3+1];
				ptr12[j*3+2] = ptr2[j*3+2];
			}
		}
	}
}


void Utils::initMatf(Mat& m, float v)
{
	for (int i = 0; i < m.rows; i++)
	{
		float* ptr = m.ptr<float>(i);
		for (int j = 0; j < m.cols; j++)
		{
			ptr[j] = v;
		}
	}
}

void Utils::initMat1u(Mat& m, int v)
{
	for (int i = 0; i < m.rows; i++)
	{
		uchar* ptr = m.ptr<uchar>(i);
		for (int j = 0; j < m.cols; j++)
			ptr[j] = v;
		
	}

}

void Utils::initMat3u(Mat& m, int v)
{
	for (int i = 0; i < m.rows; i++)
	{
		uchar* ptr = m.ptr<uchar>(i);
		for (int j = 0; j < m.cols; j++)
		{
			ptr[j*3] = v;
			ptr[j*3+1] = v;
			ptr[j*3+2] = v;
		}
	}

}

void Utils::convertXnDepthPixelToFrame(const XnDepthPixel *P, Mat& M)
{
	unsigned short mapping[MAX_DEPTH];
	for( int i=0; i<MAX_DEPTH; i++) // for visualisation
		mapping[i] = 255.0*powf(1.0-((float)i/MAX_DEPTH),3.0); // entirely arbitrary


	uchar *imagePtr = (uchar*)M.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		int charVal = mapping[P[y]];
		imagePtr[3*y]   = charVal;
		imagePtr[3*y+1] = charVal;
		imagePtr[3*y+2] = charVal;
	}
}

void Utils::convertXnRGB24PixelToFrame(const XnRGB24Pixel *P, Mat& M)
{
	uchar *imagePtr = (uchar*)M.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		imagePtr[3*y]   = P->nBlue;
		imagePtr[3*y+1] = P->nGreen;
		imagePtr[3*y+2] = P->nRed;
		P++;
	}

	/*for (int i = 0; i < M.rows; i++)
	{
		uchar* ptr = M.ptr<uchar>(i);
		for (int j = 0; j < M.cols; j++)
		{
			ptr[j*3] = P->nBlue;
			ptr[j*3+1] = P->nGreen;
			ptr[j*3+2] = P->nRed;
		}
	}*/
}

void Utils::copyDepthMap(const XnDepthPixel* depthMapIn, XnDepthPixel* depthMapOut)
{
	for (int y=0; y<XN_VGA_Y_RES; y++)
	{
		for (int x=0; x<XN_VGA_X_RES; x++)
		{
			depthMapOut[y * XN_VGA_X_RES + x] = depthMapIn[y * XN_VGA_X_RES + x];	
		}
	}

}


Scalar* Utils::getRGBRandomColor()
{
	Scalar *color = new Scalar();
	
	color->val[0] = (int)rand() % 255 + 1;
	color->val[1] = (int)rand() % 255 + 1;
	color->val[2] = (int)rand() % 255 + 1;
	return color;
}

void Utils::ConvertXnRGB24PixelToFrame(const XnRGB24Pixel *P, Mat M)
{
	uchar *imagePtr = (uchar*)M.data;
	for (int y=0; y<XN_VGA_Y_RES*XN_VGA_X_RES; y++)
	{
		imagePtr[3*y]   = P->nBlue;
		imagePtr[3*y+1] = P->nGreen;
		imagePtr[3*y+2] = P->nRed;
		P++;
	}
}

string Utils::convertInt(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}