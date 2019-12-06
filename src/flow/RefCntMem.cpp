///////////////////////////////////////////////////////////////////////////
//
// NAME
//  RefCntMem.cpp -- reference-counted heap memory
//
// SEE ALSO
//  RefCntMem.h         definition and explanation of this class
//
// Copyright © Richard Szeliski, 2001.
// See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

#include "imageio/RefCntMem.h"

CRefCntMem::CRefCntMem()
{
    // Default constructor
    m_ptr = 0;
}

CRefCntMem::~CRefCntMem()
{
    // Destructor
    DecrementCount();
    m_ptr = 0;      // not necessary, just for debugging
}

void CRefCntMem::DecrementCount()
{
    // Decrement the reference count and delete if done
    if (m_ptr)
    {
        m_ptr->m_refCnt -= 1;
        if (m_ptr->m_refCnt == 0)
        {
            if (m_ptr->m_deleteWhenDone)
            {
                if (m_ptr->m_delFn)
                    m_ptr->m_delFn(m_ptr->m_memory);
                else
                    delete (double *) m_ptr->m_memory;
            }
            delete m_ptr;
        }
    }
}

void CRefCntMem::IncrementCount()
{
    // Increment the reference count
    if (m_ptr)
    {
        m_ptr->m_refCnt += 1;
    }
}

CRefCntMem::CRefCntMem(const CRefCntMem& ref)
{
    // Copy constructor
    m_ptr = 0;
    (*this) = ref;      // use assignment operator
}

CRefCntMem& CRefCntMem::operator=(const CRefCntMem& ref)
{
    // Assignment
    DecrementCount();   // if m_ptr exists, no longer pointing to it
    m_ptr = ref.m_ptr;
    IncrementCount();
    return *this;
}

void CRefCntMem::ReAllocate(int nBytes, void *memory, bool deleteWhenDone,
                            void (*deleteFunction)(void *ptr))
{
    // Allocate/deallocate memory
    DecrementCount();
    if (memory)
    {
        m_ptr = new CRefCntMemPtr;
        m_ptr->m_nBytes = nBytes;
        m_ptr->m_memory = memory;
        m_ptr->m_deleteWhenDone = deleteWhenDone;
        m_ptr->m_refCnt = 1;
        m_ptr->m_delFn = deleteFunction;
    }
    else
        m_ptr = 0;  // don't bother storing pointer to null memory
}

int CRefCntMem::NBytes()
{
    // Number of stored bytes
    return (m_ptr) ? m_ptr->m_nBytes : 0;
}

bool CRefCntMem::InBounds(int index)
{
    // Check if index is in bounds
    return (m_ptr && 0 <= index && index < m_ptr->m_nBytes);
}

void* CRefCntMem::Memory()
{
    // Pointer to allocated memory
    return (m_ptr) ? m_ptr->m_memory : 0;
}
