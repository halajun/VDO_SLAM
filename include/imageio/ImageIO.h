///////////////////////////////////////////////////////////////////////////
//
// NAME
//  ImageIO.h -- image file input/output
//
// DESCRIPTION
//  Read/write image files, potentially using an interface to an
//  external package.
//
//  You can either pass and empty (unitialized) image to ReadImage,
//  or one you have already allocated (with a specific pixel type).
//  
//  If you don't initialize the image, the type of the returned image
//  (e.g., 1 band vs. 4 band) will be determined by the image file.
//  If you do initialize the image, it will be re-allocated if necessary,
//  and the data will be coerced into the type you specified.
//
//  File formats currently supported:
//  - a subset of Targa
//  - PGM (1 band) and PPM (4 band)
//  - PMF (multiband float) - homegrown, non-standard
//        (PFM already taken by postscript font maps)
//  - PNG (requires ImageIOpng.cpp, and pnglib and zlib packages)
//
// SEE ALSO
//  ImageIO.cpp          implementation
//  ImageIOpng.cpp       png reader/writer
//
// Copyright © Richard Szeliski and Daniel Scharstein, 2001.
// See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

void ReadImage (CImage& img, const char* filename);
void WriteImage(CImage& img, const char* filename);

void ReadImageVerb (CImage& img, const char* filename, int verbose);
void WriteImageVerb(CImage& img, const char* filename, int verbose);
