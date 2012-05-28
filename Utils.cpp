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