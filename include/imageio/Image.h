///////////////////////////////////////////////////////////////////////////
//
// NAME
//  Image.h -- a simple reference-counted image structure
//
// DESCRIPTION
//  The CImage class is used to create and access weakly typed
//  two-dimensional images.  
//
//  The templated CImageOf<T> classes are used to create strongly typed images.
//  The currently supported pixel types are:
//      unsigned char, int, and float.
//
//  The images can have an arbitrary width, height, and also an arbitrary
//  number of bands (channels) per pixel.  For example, traditional RGBA
//  images can be represented using a 4-channel unsigned_8 image.
//
//  Images are normally allocated on the stack (NOT on the heap, i.e.,
//  "new Image" should not be used).  They can be freely returned from
//  functions and put into other data structures.  Assignment and copy
//  construction share memory (to copy pixel values from one image to
//  another one, use CopyPixels()).
//
// SEE ALSO
//  Image.cpp           implementation
//  RefCntMem.h         reference-counted memory object used by CImage
//
// Copyright � Richard Szeliski, 2001.
// See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////


namespace std {}
using namespace std;

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "RefCntMem.h"

#ifdef WIN32
#include <typeinfo.h>
#else
#include <typeinfo>
#define __max(a,b)  (((a) > (b)) ? (a) : (b))
#define __min(a,b)  (((a) < (b)) ? (a) : (b))
#endif

#include <float.h>
#ifndef FLT_MAX
#define FLT_MAX         3.402823466e+38F        /* max value */
#define FLT_MIN         1.175494351e-38F        /* min positive value */
#endif

typedef unsigned char uchar;


// Shape of an image: width x height x nbands

struct CShape
{
    int width, height;      // width and height in pixels
    int nBands;             // number of bands/channels
    
    // Constructors and helper functions 
    CShape(void) : width(0), height(0), nBands(0) {}
    CShape(int w, int h, int nb) : width(w), height(h), nBands(nb) {}
    bool InBounds(int x, int y);            // is given pixel address valid?
    bool InBounds(int x, int y, int band);  // is given pixel address valid?
    bool operator==(const CShape& ref);     // are two shapes the same?
    bool SameIgnoringNBands(const CShape& ref); // " ignoring the number of bands?
    bool operator!=(const CShape& ref);     // are two shapes not the same?
};

inline bool CShape::InBounds(int x, int y)
{
    // Is given pixel address valid?
    return (0 <= x && x < width &&
            0 <= y && y < height);
}

inline bool CShape::InBounds(int x, int y, int b)
{
    // Is given pixel address valid?
    return (0 <= x && x < width &&
            0 <= y && y < height &&
            0 <= b && b < nBands);
}


// Padding (border) mode for neighborhood operations like convolution

enum EBorderMode
{
    eBorderZero         = 0,    // zero padding
    eBorderReplicate    = 1,    // replicate border values
    eBorderReflect      = 2,    // reflect border pixels
    eBorderCyclic       = 3     // wrap pixel values
};

// Image attributes

struct CImageAttributes
{
    int alphaChannel;       // which channel contains alpha (for compositing)
    int origin[2];          // x and y coordinate origin (for some operations)
    EBorderMode borderMode; // border behavior for neighborhood operations...
    // char colorSpace[4];     // RGBA, YUVA, etc.: not currently used
};


// Generic (weakly typed) image

class CImage : public CImageAttributes
{
public:
    CImage(void);               // default constructor
    CImage(CShape s, const type_info& ti, int bandSize);
    // uses system-supplied copy constructor, assignment operator, and destructor

    void ReAllocate(CShape s, const type_info& ti, int bandSize,
                    void *memory, bool deleteWhenDone, int rowSize);
    void ReAllocate(CShape s, const type_info& ti, int bandSize,
                    bool evenIfSameShape = false);
    void DeAllocate(void);      // release the memory & set to default values

    CShape Shape(void)              { return m_shape; }
    const type_info& PixType(void)  { return *m_pTI; }
    int BandSize(void)              { return m_bandSize; }

    void* PixelAddress(int x, int y, int band);

    void SetSubImage(int xO, int yO, int width, int height);   // sub-image sharing memory

protected:
    void SetPixels(void *val_ptr);  // Fill the image with a value

private:
    void SetDefaults(void); // set internal state to default values

    CShape m_shape;         // image shape (dimensions)
    const type_info* m_pTI; // pointer to type_info class
    int m_bandSize;         // size of each band in bytes
    int m_pixSize;          // stride between pixels in bytes
    int m_rowSize;          // stride between rows in bytes
    char* m_memStart;       // start of addressable memory
    CRefCntMem m_memory;    // reference counted memory
public:
    int alphaChannel;       // which channel contains alpha (for compositing)
};

inline void* CImage::PixelAddress(int x, int y, int band)
{
    // This could also go into the implementation file (CImage.cpp):
    return (void *) &m_memStart[y * m_rowSize + x * m_pixSize + band * m_bandSize];
}


//  Strongly typed image

template <class T>
class CImageOf : public CImage
{
public:
    CImageOf(void);
    CImageOf(CShape s);
    CImageOf(int width, int height, int nBands);
    // uses system-supplied copy constructor, assignment operator, and destructor

    void ReAllocate(CShape s, bool evenIfSameShape = false);
    void ReAllocate(CShape s, T *memory, bool deleteWhenDone, int rowSize);

    T& Pixel(int x, int y, int band);

    CImageOf SubImage(int x, int y, int width, int height);   // sub-image sharing memory

    void FillPixels(T val);     // fill the image with a constant value
    void ClearPixels(void);     // fill the image with a 0 value

    T MinVal(void);     // minimum allowable value (for clipping)
    T MaxVal(void);     // maximum allowable value (for clipping)
};

//  These are defined inline so user-defined image types can be supported:

template <class T>
inline CImageOf<T>::CImageOf(void) :
CImage(CShape(), typeid(T), sizeof(T)) {}

template <class T>
inline CImageOf<T>::CImageOf(CShape s) :
CImage(s, typeid(T), sizeof(T)) {}

template <class T>
inline CImageOf<T>::CImageOf(int width, int height, int nBands) :
CImage(CShape(width, height, nBands), typeid(T), sizeof(T)) {}

template <class T>
inline void CImageOf<T>::ReAllocate(CShape s, bool evenIfSameShape)
{
    CImage::ReAllocate(s, typeid(T), sizeof(T), evenIfSameShape);
}

template <class T>
inline void CImageOf<T>::ReAllocate(CShape s, T *memory,
                                    bool deleteWhenDone, int rowSize)
{
    CImage::ReAllocate(s, typeid(T), sizeof(T), memory, deleteWhenDone, rowSize);
}
    
template <class T>
inline T& CImageOf<T>::Pixel(int x, int y, int band)
{
    return *(T *) PixelAddress(x, y, band);
}

template <class T>
inline CImageOf<T> CImageOf<T>::SubImage(int x, int y, int width, int height)
{
    // sub-image sharing memory
    CImageOf<T> retval = *this;
    retval.SetSubImage(x, y, width, height);
    return retval;
}

template <class T>
inline void CImageOf<T>::FillPixels(T val)
{
    // fill the image with a constant value
    SetPixels(&val);
}

template <class T>
inline void CImageOf<T>::ClearPixels(void)
{
    // fill the image with a 0 value
    T val = 0;
    FillPixels(val);
}

// Commonly used types (supported in type conversion routines):

typedef CImageOf<uchar> CByteImage;
typedef CImageOf<int>   CIntImage;
typedef CImageOf<float> CFloatImage;

// Color pixel support

template <class PixType>
struct RGBA
{
    PixType B, G, R, A;     // A channel is highest one
};
