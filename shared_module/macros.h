#pragma once

#include <tbb/tbbmalloc_proxy.h>

// EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif

// IMPORT
#ifndef IMPORT_API
#if defined(_MSC_VER)
#define IMPORT_API __declspec(dllimport)
#else
#define IMPORT_API __attribute__((visibility("default")))
#endif
#endif

// EXPORT
#ifndef EXPORT_API
#if defined(_MSC_VER)
// MSVC linker trims symbols, the 'dllexport' attribute prevents this.
// But we are not archiving DLL files with SHIPPING_ONE_ARCHIVE mode.
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API __attribute__((visibility("default")))
#endif
#endif

#ifndef API
#ifdef SHARED_MODULE
#define API EXPORT_API
#elif
#define API IMPORT_API
#endif
#endif

#ifndef EXTERN_C_BEGIN
#ifdef __cplusplus
#define EXTERN_C_BEGIN extern "C" {
#else
#define EXTERN_C_BEGIN
#endif
#endif

#ifndef EXTERN_C_END
#ifdef __cplusplus
#define EXTERN_C_END }
#else
#define EXTERN_C_END
#endif
#endif