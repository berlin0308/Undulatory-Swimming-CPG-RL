#ifndef __GAUSS_H
#define __GAUSS_H

#include "../matrix/matrix.h"

typedef void GaussCallback (double x, Vector differential, void *data);

Vector
gauss_quadrature (double x0,
                  double x1,
                  GaussCallback callback,
                  void   *data,
                  Vector  differential,
                  Vector  out,
                  int     n);

#endif
