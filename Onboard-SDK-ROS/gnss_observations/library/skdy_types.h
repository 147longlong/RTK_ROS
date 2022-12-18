

#ifndef SKDY_TYPES_H__
#define SKDY_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

#ifndef SKDY_CHAR_T
#define SKDY_CHAR_T
    typedef char skdy_char_t;
#endif

#ifndef SKDY_INT8_T
#define SKDY_INT8_T
    typedef signed char skdy_int8_t;
#endif

#ifndef SKDY_UINT8_T
#define SKDY_UINT8_T
    typedef unsigned char skdy_uint8_t;
#endif

#ifndef SKDY_INT16_T
#define SKDY_INT16_T
    typedef signed short skdy_int16_t;
#endif

#ifndef SKDY_UINT16_T
#define SKDY_UINT16_T
    typedef unsigned short skdy_uint16_t;
#endif

#ifndef SKDY_INT32_t
#define SKDY_INT32_t
    typedef signed int skdy_int32_t;
#endif

#ifndef SKDY_UINT32_T
#define SKDY_UINT32_T
    typedef unsigned int skdy_uint32_t;
#endif

#ifndef SKDY_INT64_T
#define SKDY_INT64_T
    typedef signed long long skdy_int64_t;
#endif

#ifndef SKDY_UINT64_T
#define SKDY_UINT64_T
    typedef unsigned long long skdy_uint64_t;
#endif

#ifndef SKDY_FLOAT32_T
#define SKDY_FLOAT32_T
    typedef float skdy_float32_t;
#endif

#ifndef SKDY_FLOAT64_T
#define SKDY_FLOAT64_T
    typedef double skdy_float64_t;
#endif

#ifndef SKDY_BOOL_T
#define SKDY_BOOL_T
    typedef bool skdy_bool_t;
#endif


#ifdef __cplusplus
}
#endif

#endif
