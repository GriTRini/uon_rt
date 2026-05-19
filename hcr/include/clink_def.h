//------------------------------------------------------------------------------
//
// ControLINK-API
// Copyright ⓒ Hanwha Robotics. All rights reserved.
// (https://www.hanwharobotics.co.kr/)
//
// Seokhwan Kim    (seokhwan at hanwha dot com)
// Kazuma   Fukuda (k.fukuda at hanwha dot com)
// Kiho     Cho    (kiho0520 at hanwha dot com)
// Incheol  Jeong  (ic.jeong at hanwha dot com)
// 
//------------------------------------------------------------------------------

#ifndef __CLINK_DEF_H__
#define __CLINK_DEF_H__

#include "clink_config.h"

#if defined(CLINK_WIN)
    #ifdef _WINDLL
        #define CLINK_EXPORT __declspec(dllexport)
        #define CLINK_IMPORT __declspec(dllimport)
    #endif
    #pragma warning( disable : 4251 4996 26812 )
#endif

#ifndef CLINK_EXPORT
    #define CLINK_EXPORT
#endif

#ifdef CLINK_IAR
    #define _DLIB_FILE_DESCRIPTOR 1
#endif

#define CLINK_NULL_PTR nullptr

#include <cstddef>
#include <stdint.h>

#ifndef CLINK_BYTE_T_DEFINED
    typedef char              clink_byte_t;
    #define CLINK_BYTE_T_DEFINED
#endif

#ifndef CHAR_T_DEFINED
    typedef char               char_t;
    #define CHAR_T_DEFINED
#endif 

#ifndef UCHAR_T_DEFINED
    typedef unsigned char      uchar_t;
    #define UCHAR_T_DEFINED
#endif 

#ifndef __USE_CONTROL_PORT_LIB__
    /* 하위 Library로서 Control Port를 사용하는 경우 Control Port에서 있는 정의와 중복되기 때문에 #ifdef 설정을 허용한다. */
#ifndef FLOAT32_T_DEFINED
    typedef float              float32_t;
    #define FLOAT32_T_DEFINED
#endif 

#ifndef FLOAT64_T_DEFINED
    typedef double             float64_t;
    #define FLOAT64_T_DEFINED
#endif 
#endif	//  __USE_CONTROL_PORT_LIB__

#ifdef CLINK_USE_SINGLE
    typedef float32_t clink_float_t;
#else
    typedef float64_t clink_float_t;
#endif


#ifdef CLINK_USE_INT32
    typedef int32_t  clink_int_t;
    typedef uint32_t clink_uint_t;
#else
    typedef int64_t  clink_int_t;
    typedef uint64_t clink_uint_t;
#endif


#ifdef CLINK_USE_SINGLE
    #define CLINK_FLOAT_ZERO  0.F
    #define CLINK_FLOAT_ONE   1.0F
#else
    #define CLINK_FLOAT_ZERO  0.0
    #define CLINK_FLOAT_ONE   1.0
#endif

#ifdef CLINK_USE_INT32
    #define CLINK_INT_ZERO 0
#else
    #define CLINK_INT_ZERO 0
#endif

#define CLINK_SAFE_DELETE(ptr) \
    if (CLINK_NULL_PTR != ptr) \
        {delete ptr; ptr = CLINK_NULL_PTR; }
#define CLINK_SAFE_DELETE_ARRAY(ptr) \
    if (CLINK_NULL_PTR != ptr) \
        {delete [] ptr; ptr = CLINK_NULL_PTR; }


#endif // __CLINK_API_DEF_H__
