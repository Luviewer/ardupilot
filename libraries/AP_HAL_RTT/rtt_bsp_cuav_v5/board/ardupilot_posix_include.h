/* Force POSIX declarations for ArduPilot RTT build. */
#ifndef _ARDUPILOT_POSIX_INCLUDE_H_
#define _ARDUPILOT_POSIX_INCLUDE_H_
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include <string.h>
#include <strings.h>
#ifdef __cplusplus
#include <cstdio>
#endif
#ifndef M_SQRT1_2
#define M_SQRT1_2 0.707106781186547524401
#endif
#ifdef __cplusplus
extern "C" int ffs(int);
extern "C" int asprintf(char **, const char *, ...);
extern "C" void *memmem(const void *, size_t, const void *, size_t);
extern "C" size_t strnlen(const char *s, size_t maxlen);
extern "C" char *strdup(const char *s);
#endif
#endif
