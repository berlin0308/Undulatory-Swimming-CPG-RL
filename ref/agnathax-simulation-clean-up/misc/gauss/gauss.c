#include "gauss.h"

#define GAUSS_N_POINTS 6

const double gauss_positions[] = {0.238619186083197,
                                  0.661209386466265,
                                  0.932469514203152,
                                  -0.238619186083197,
                                  -0.661209386466265,
                                  -0.932469514203152};

const double gauss_weights[] = {0.467913934572691,
                                0.360761573048139,
                                0.171324492379170,
                                0.467913934572691,
                                0.360761573048139,
                                0.171324492379170};

Vector
gauss_quadrature (double x0,
                  double x1,
                  GaussCallback callback,
                  void   *data,
                  Vector  differential,
                  Vector  out,
                  int     n)
{
	int i;

	vector_zeros (out, n);

	for (i = 0; i < GAUSS_N_POINTS; ++i)
	{
		double det_J = (x1 - x0) / 2.0; /* gauss interval width = 2, user interval width = x1 - x0 */
		double x = x0 + (gauss_positions[i] + 1) / 2.0 * (x1 - x0);

		callback (x, differential, data);

		vector_multiply (differential, gauss_weights[i] * det_J, differential, n);
		vector_sum (differential, out, out, n);
	}

	return out;
}
