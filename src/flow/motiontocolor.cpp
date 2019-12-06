#include <cmath>
#include "imageio/imageLib.h"
#include "flow/flowIO.h"
#include "flow/colorcode.h"
#include "flow/motiontocolor.h"

void MotionToColor(CFloatImage motim, CByteImage &colim, float maxmotion)
{
    CShape sh = motim.Shape();
    int width = sh.width, height = sh.height;
    colim.ReAllocate(CShape(width, height, 3));
    int x, y;
    // determine motion range:
    float maxx = -999, maxy = -999;
    float minx =  999, miny =  999;
    float maxrad = -1;
    for (y = 0; y < height; y++) {
	for (x = 0; x < width; x++) {
	    float fx = motim.Pixel(x, y, 0);
	    float fy = motim.Pixel(x, y, 1);
	    if (unknown_flow(fx, fy))
		continue;
	    maxx = __max(maxx, fx);
	    maxy = __max(maxy, fy);
	    minx = __min(minx, fx);
	    miny = __min(miny, fy);
	    float rad = std::sqrt(fx * fx + fy * fy);
	    maxrad = __max(maxrad, rad);
	}
    }
//    printf("max motion: %.4f  motion range: u = %.3f .. %.3f;  v = %.3f .. %.3f\n",
//	   maxrad, minx, maxx, miny, maxy);


    if (maxmotion > 0) // i.e., specified on commandline
	maxrad = maxmotion;

    if (maxrad == 0) // if flow == 0 everywhere
	maxrad = 1;

//    if (verbose)
//	fprintf(stderr, "normalizing by %g\n", maxrad);

    for (y = 0; y < height; y++) {
	for (x = 0; x < width; x++) {
	    float fx = motim.Pixel(x, y, 0);
	    float fy = motim.Pixel(x, y, 1);
	    uchar *pix = &colim.Pixel(x, y, 0);
	    if (unknown_flow(fx, fy)) {
		pix[0] = pix[1] = pix[2] = 0;
	    } else {
		computeColor(fx/maxrad, fy/maxrad, pix);
	    }
	}
    }
}