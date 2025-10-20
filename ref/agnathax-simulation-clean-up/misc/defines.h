#ifndef __MISC_DEFINES_H
#define __MISC_DEFINES_H

#ifndef MIN
#define MIN(X, Y)                  ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
#define MAX(X, Y)                  ((X) > (Y) ? (X) : (Y))
#endif

#ifndef CLIP
#define CLIP(X, X0, X1)            ((X) < (X0) ? (X0) : (X) > (X1) ? (X1) : (X))
#endif

#ifndef INTERP
#define INTERP(MIN, MAX, FRACTION) ((MIN) + (FRACTION) * ((MAX) - (MIN)))
#endif

#ifndef RAND
#define RAND(A0, A1)               ((A0) + rand() * 1.0 / RAND_MAX * ((A1) - (A0)))
#endif

#ifndef FABS
#define FABS(A0)                   ((A0) < 0 ? -(A0) : (A0))
#endif

#if !defined(STRINGIFY) && !defined(STRINGIFY)
#define _STRINGIFY(X)  #X
#define STRINGIFY(X)   _STRINGIFY(X)
#endif

#endif
