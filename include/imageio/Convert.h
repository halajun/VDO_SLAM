///////////////////////////////////////////////////////////////////////////
//
// NAME
//  Convert.h -- convert between image types, copy images, select bands
//
// DESCRIPTION
//  This file defines a number of conversion/copying utilities:
//
//  void ScaleAndOffset(CImageOf<T1>& src, CImageOf<T2>& dst,
//                       float scale, float offset);
//      -- scale and offset one image into another (optionally convert type)
//
//  void CopyPixels(CImageOf<T1>& src, CImageOf<T2>& dst);
//      -- convert pixel types or just copy pixels from src to dst
//
//  CImageOf<T> ConvertToRGBA(CImageOf<T> src);
//      -- convert from gray (1-band) image to RGBA (alpha == 255)
//
//  CImageOf<T> ConvertToGray(CImageOf<T> src);
//      -- convert from RGBA (4-band) image to gray, using Y formula,
//          Y = 0.212671 * R + 0.715160 * G + 0.072169 * B
//
//  void BandSelect(CImageOf<T>& src, CImageOf<T>& dst, int sBand, int dBand);
//      -- copy the sBand from src into the dBand in dst
//
//  The ScaleAndOffset and CopyPixels routines will reallocate dst if it
//  doesn't conform in shape to src.  So will BandSelect, except that the
//  number of bands in src and dst is allowed to differ (if dst is
//  unitialized, it will be set to a 1-band image).
//
// PARAMETERS
//  src                 source image
//  dst                 destination image
//  scale               floating point scale value  (1.0 = no change)
//  offset              floating point offset value (0.0 = no change)
//  sBand               source band (0...)
//  dBand               destination band (0...)
//
// SEE ALSO
//  Convert.cpp         implementation
//  Image.h             image class definition
//
// Copyright © Richard Szeliski, 2001.
// See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

template <class T1, class T2>
void ScaleAndOffsetLine(T1* src, T2* dst, int n,
                        float scale, float offset,
                        T2 minVal, T2 maxVal);

template <class T1, class T2>
void ScaleAndOffset(CImageOf<T1>& src, CImageOf<T2>& dst,
                    float scale, float offset);

template <class T1, class T2>
void CopyPixels(CImageOf<T1>& src, CImageOf<T2>& dst)
{
    ScaleAndOffset(src, dst, 1.0f, 0.0f);
}

template <class T>
CImageOf<T> ConvertToRGBA(CImageOf<T> src);

template <class T>
CImageOf<T> ConvertToGray(CImageOf<T> src);

template <class T>
void BandSelect(CImageOf<T>& src, CImageOf<T>& dst, int sBand, int dBand);
