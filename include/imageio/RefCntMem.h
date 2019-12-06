///////////////////////////////////////////////////////////////////////////
//
// NAME
//  RefCntMem.h -- reference-counted heap memory
//
// DESCRIPTION
//  The CRefCntMem class is used to manage reference-counted
//  dynamically allocatable memory.  This memory can be shared
//  across many instances of the class object, provided they
//  were created from each other through copy construction or assignment.
//
//  Using the class in a large memory object class such as CImage allows
//  the including class to achieve a similar kind of memory sharing as
//  is found in garbage collected languages such as Java and C#.
//
// SEE ALSO
//  RefCntMem.cpp       implementation
//  Image.h             class that uses a CRefCntMem object
//
// Copyright © Richard Szeliski, 2001.
// See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

struct CRefCntMemPtr         // shared component of reference counted memory
{
    void *m_memory;         // allocated memory
    int m_refCnt;           // reference count
    int m_nBytes;           // number of bytes
    bool m_deleteWhenDone;  // delete memory when ref-count drops to 0
    void (*m_delFn)(void *ptr); // optional delete function
};

class CRefCntMem            // reference-counted memory allocator
{
public:
    CRefCntMem(void);           // default constructor
    CRefCntMem(const CRefCntMem& ref);  // copy constructor
    ~CRefCntMem(void);          // destructor
    CRefCntMem& operator=(const CRefCntMem& ref);  // assignment

    void ReAllocate(int nBytes, void *memory, bool deleteWhenDone,
                    void (*deleteFunction)(void *ptr) = 0);
        // allocate/deallocate memory
    int NBytes(void);           // number of stored bytes
    bool InBounds(int i);       // check if index is in bounds
    void* Memory(void);         // pointer to allocated memory
private:
    void DecrementCount(void);  // decrement the reference count and delete if done
    void IncrementCount(void);  // increment the reference count
    CRefCntMemPtr *m_ptr;       // shared reference-counted memory pointer
};

