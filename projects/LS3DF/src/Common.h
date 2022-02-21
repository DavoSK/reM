//result codes
#ifndef __ENG_3DI_
#define __ENG_3DI_
#pragma once

typedef long I3D_RESULT;
enum 
{
    I3D_OK,
    I3D_DONE,
    I3DERR_NOFILE = 0x80000001,//unable to open file
    I3DERR_OUTOFMEM = 0x80000002, //out of memory
    I3DERR_TEXTURESNOTSUPPORTED = 0x80000003,
    I3DERR_GENERIC = 0x80000006,  //internal error
    I3DERR_OBJECTNOTFOUND = 0x80000007, //find/delete - no such object in list
    I3DERR_INVALIDPARAMS = 0x80000008,
    I3DERR_CANTSETSECTOR = 0x8000000b,
    I3DERR_NOTINITIALIZED,
    I3DERR_FILECORRUPTED,
    I3DERR_NOFILE1,            //additional file (opacity map)
    I3DERR_CYCLICLINK,         //the linking would cause cyclic hierarchy
    I3DERR_CANCELED,           //operation has been cancelled by user (enumeration, loading, lightmap computation, etc)
    I3DERR_UNSUPPORTED,
};

typedef I3D_RESULT LS3D_RESULT;

#ifndef I3D_SUCCESS
#define I3D_SUCCESS(n) ((I3D_RESULT)n>=0)
#endif

#ifndef I3D_FAIL
#define I3D_FAIL(n) ((I3D_RESULT)n<0)
#endif

#endif