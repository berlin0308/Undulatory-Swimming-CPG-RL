#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "matrix.h"

Vector
vector_zeros (Vector v,
              int    n)
{
	return (Vector) memset (v, 0, n * sizeof (VectorElement));
}

Vector
vector3_zeros (Vector3 v)
{
	return vector_zeros (v, 3);
}

Matrix
matrix3_zeros (Matrix3 m)
{
	return (Matrix) vector_zeros ((Vector) m, 3 * 3);
}

Matrix
matrix6_zeros (Matrix6 m)
{
	return (Matrix) vector_zeros ((Vector) m, 6 * 6);
}

Vector
vector_copy (Vector v_out,
             Vector v_in,
             int    n)
{
	return (Vector) memcpy (v_out, v_in, n * sizeof (VectorElement));
}

Vector
vector3_map (Vector3       v,
             VectorMapFunc f,
             Vector3       v_out)
{
	int i;

	for (i = 0; i < 3; ++i)
	{
		v_out[i] = f (v[i]);
	}

	return v_out;
}



Vector
vector_multiply (Vector v_in,
                 double a,
                 Vector v_out,
                 int    n)
{
	int i;

	for (i = 0; i < n; ++i)
	{
		v_out[i] = a * v_in[i];
	}

	return v_out;
}

Vector
vector3_multiply (Vector3 v_in,
                  double  a,
                  Vector3 v_out)
{
	return vector_multiply (v_in, a, v_out, 3);
}




double
vector_dot_product (Vector v1,
                    Vector v2,
                    int    n)
{
	int i;

	double sum = 0;
	for (i = 0; i < n; ++i)
	{
		sum += v1[i] * v2[i];
	}

	return sum;
}

double
vector3_dot_product (Vector3 v1,
                     Vector3 v2)
{
	return vector_dot_product (v1, v2, 3);
}


Matrix
matrix3_multiply (Matrix3 m_in,
                  double  a,
                  Matrix3 m_out)
{
	return matrix_multiply (m_in, a, m_out, 3, 3);
}

Matrix
matrix6_multiply (Matrix6 m_in,
                  double  a,
                  Matrix6 m_out)
{
	return matrix_multiply (m_in, a, m_out, 6, 6);
}

Vector
matrix3_product_vector (Matrix3 m_in,
                        Vector3 v_in,
                        Vector3 v_out)
{
	Vector3 v_temp;
	return matrix_product_vector_temp (m_in, v_in, v_out, 3, 3, v_temp);
}




Vector
matrix3_product_vector6 (Matrix3 m_in,
                         Vector6 v_in,
                         Vector6 v_out)
{
	Vector3 v_temp;

	matrix_product_vector_temp (m_in, &v_in[0], &v_out[0], 3, 3, v_temp);
	matrix_product_vector_temp (m_in, &v_in[3], &v_out[3], 3, 3, v_temp);

	return v_out;
}

Vector
matrix6_product_vector (Matrix6 m_in,
                        Vector6 v_in,
                        Vector6 v_out)
{
	Vector6 v_temp;
	return matrix_product_vector_temp (m_in, v_in, v_out, 6, 6, v_temp);
}

Matrix
matrix6_product_matrix6 (Matrix6 m1,
                         Matrix6 m2,
                         Matrix6 m_out)
{
	Matrix6 m_temp;
	int row, column, i;

	for (row = 0; row < 6; ++row)
	{
		for (column = 0; column < 6; ++column)
		{
			m_temp[row][column] = 0;

			for (i = 0; i < 6; ++i)
			{
				m_temp[row][column] += m1[row][i] * m2[i][column];
			}
		}
	}

	return (Matrix) memcpy (m_out, m_temp, sizeof (Matrix6));
}

Matrix
matrix3_product_matrix3 (Matrix3 m1,
                         Matrix3 m2,
                         Matrix3 m_out)
{
	Matrix3 m_temp;
	int row, column, i;

	for (row = 0; row < 3; ++row)
	{
		for (column = 0; column < 3; ++column)
		{
			m_temp[row][column] = 0;

			for (i = 0; i < 3; ++i)
			{
				m_temp[row][column] += m1[row][i] * m2[i][column];
			}
		}
	}

	return (Matrix) memcpy (m_out, m_temp, sizeof (Matrix3));
}

/* Returns M such that M x = v cross x */
Matrix
vector3_antisymmetric_matrix (Vector3 v,
                              Matrix3 m_out)
{
	Matrix3 antisym = {{ 0     , -v[2] , v[1]  },
	                   { v[2]  , 0     , -v[0] },
	                   { -v[1] , v[0]  , 0     }};

	return (Matrix) memcpy (m_out, antisym, sizeof (Matrix3));
}

void
vector_print (Vector v,
              int     n)
{
	int i;

	for (i = 0; i < n; ++i)
	{
		printf ("\t%f\n", v[i]);
	}
	puts ("");
}

void
vector3_print (Vector3 v)
{
	vector_print (v, 3);
}

void
matrix3_print (Matrix3 m)
{
	matrix_print (m, 3, 3);
}

Vector
matrixn_product_vector3 (Matrix3  m_in,
                         Vector3 v_in,
                         Vector3 v_out,
                         int     n)
{
	Vector3 v_temp;

	int i, j;

	for (i = 0; i < 3; ++i)
	{
		v_temp[i] = 0;

		for (j = 0; j < 3; ++j)
		{
			v_temp[i] += MATRIX_IJ (m_in, i, j, n) * v_in[j];
		}
	}

	return (Vector) memcpy (v_out, v_temp, sizeof (Vector3));
}

/* Returns 6x6 twist transformation matrix for a translation along v. */
/* Transpose to get the wrench transformation matrix. */
Matrix
vector3_translation_transformation_matrix (Vector3 v,
                                           Matrix6 m_out)
{
	Matrix3 v_cross;
	int i, j;

	vector3_antisymmetric_matrix (v, v_cross);

	matrix_identity (m_out, 6);

	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			m_out[i][j + 3] = -v_cross[i][j];
		}
	}

	return (Matrix) m_out;
}

Matrix
matrix3_transpose (Matrix3 m,
                   Matrix3 mT)
{
	Matrix3 m_temp;
	int i, j;

	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			m_temp[i][j] = m[j][i];
		}
	}

	return (Matrix) memcpy (mT, m_temp, sizeof (Matrix3));
}

Matrix
matrix6_transpose (Matrix6 m,
                   Matrix6 mT)
{
	Matrix6 m_temp;
	int i, j;

	for (i = 0; i < 6; ++i)
	{
		for (j = 0; j < 6; ++j)
		{
			m_temp[i][j] = m[j][i];
		}
	}

	return (Matrix) memcpy (mT, m_temp, sizeof (Matrix6));
}

Vector
vector3_cross_vector (Vector3 v1,
                      Vector3 v2,
                      Vector3 v_out)
{
	Vector3 v_temp = {v1[1] * v2[2] - v1[2] * v2[1],
	                  v1[2] * v2[0] - v1[0] * v2[2],
	                  v1[0] * v2[1] - v1[1] * v2[0]};

	return (Vector) memcpy (v_out, v_temp, sizeof (Vector3));
}

Vector
vector3_times_vector (Vector3 v1,
                      Vector3 v2,
                      Vector3 v_out)
{
	int i;

	for (i = 0; i < 3; ++i)
	{
		v_out[i] = v1[i] * v2[i];
	}

	return v_out;
}

Vector
vector_sum (Vector v1,
            Vector v2,
            Vector v_out,
            int    n)
{
	int i;

	for (i = 0; i < n; ++i)
	{
		v_out[i] = v1[i] + v2[i];
	}

	return v_out;
}

Vector
vector_difference (Vector v1,
                   Vector v2,
                   Vector v_out,
                   int    n)
{
	int i;

	for (i = 0; i < n; ++i)
	{
		v_out[i] = v1[i] - v2[i];
	}

	return v_out;
}

Vector
vector3_sum (Vector3 v1,
             Vector3 v2,
             Vector3 v_out)
{
	return vector_sum (v1, v2, v_out, 3);
}

double
vector_norm (Vector v,
             int n)
{
	return sqrt (vector_dot_product (v, v, n));
}

double
vector3_norm (Vector3 v)
{
	return vector_norm (v, 3);
}

Vector
vector_normalize (Vector v_in,
                  Vector v_out,
                  int    n)
{
	double norm = vector_norm (v_in, n);

	if (norm == 0)
	{
		vector_copy (v_out, v_in, n);
	}
	else {
		int i;

		for (i = 0; i < n; ++i)
		{
			v_out[i] = v_in[i] / norm;
		}
	}

	return v_out;
}

Vector
vector3_normalize (Vector3 v_in,
                   Vector3 v_out)
{
	return vector_normalize (v_in, v_out, 3);
}

Vector
vector6_sum (Vector6 v1,
             Vector6 v2,
             Vector6 v_out)
{
	return vector_sum (v1, v2, v_out, 6);
}

Vector
vector3_difference (Vector3 v1,
                    Vector3 v2,
                    Vector3 v_out)
{
	return vector_difference (v1, v2, v_out, 3);
}

Matrix
matrix3_sum (Matrix3 m1,
             Matrix3 m2,
             Matrix3 m_out)
{
	return (Matrix) vector_sum ((Vector) m1, (Vector) m2, (Vector) m_out, 9);
}

Matrix
matrix6_sum (Matrix6 m1,
             Matrix6 m2,
             Matrix6 m_out)
{
	return (Matrix) vector_sum ((Vector) m1, (Vector) m2, (Vector) m_out, 36);
}

Matrix
vector3_tensor_product (Vector3 v1,
                        Vector3 v2,
                        Matrix3 m_out)
{
	int i, j;

	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			m_out[i][j] = v1[i] * v2[j];
		}
	}

	return (Matrix) m_out;
}

Matrix
matrix6_set_block3 (Matrix6 m,
                    Matrix3 block,
                    int     row,
                    int     column)
{
	int i, j;

	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			m[row + i][column + j] = block[i][j];
		}
	}

	return (Matrix) m;
}

Matrix
matrix6_get_block3 (Matrix6 m,
                    Matrix3 block,
                    int     row,
                    int     column)
{
	int i, j;

	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			block[i][j] = m[row + i][column + j];
		}
	}

	return (Matrix) block;
}

Matrix
matrix6_erase_block3 (Matrix6 m,
                      int     row,
                      int     column)
{
	int i, j;

	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			m[row + i][column + j] = 0;
		}
	}

	return (Matrix) m;
}

Matrix
matrix6_by_blocks3 (Matrix6 m_out,
                    Matrix3 m11,
                    Matrix3 m12,
                    Matrix3 m21,
                    Matrix3 m22)
{
	int const row_starts[4] = {0, 0, 3, 3};
	int const col_starts[4] = {0, 3, 0, 3};
	VectorElement (*block[4])[] = {(VectorElement (*)[]) m11,(VectorElement (*)[]) m12,(VectorElement (*)[]) m21,(VectorElement (*)[]) m22};
	int i;

	matrix6_zeros (m_out);

	for (i = 0; i < 4; ++i)
	{
		if (block[i])
		{
			matrix6_set_block3 (m_out, (VectorElement (*)[3])block[i], row_starts[i], col_starts[i]);
		}
	}

	return (Matrix) m_out;
}

Vector
quaternion_conjugate (Quaternion q_in,
                      Quaternion q_out)
{
	int i;
	
	q_out[0] = q_in[0];
	
	for (i = 1; i < 4; ++i)
	{
		q_out[i] = -q_in[i];
	}

	return q_out;
}

Vector
quaternion_inverse (Quaternion q_in,
                    Quaternion q_out)
{
	int i;

	double norm2 = vector_dot_product (q_in, q_in, 4);

	if (norm2 == 0)
	{
		vector_copy (q_out, q_in, 4);
	}
	else
	{
		q_out[0] = q_in[0] / norm2;

		for (i = 1; i < 4; ++i)
		{
			q_out[i] = -q_in[i] / norm2;
		}
	}

	return q_out;
}

Vector
quaternion_product_quaternion (Quaternion q1,
                               Quaternion q2,
                               Quaternion q_out)
{
	Vector3 v;
	Quaternion qtemp;

	double q_out0 = q1[0] * q2[0] - vector3_dot_product (Q_VECTOR (q1), Q_VECTOR (q2));

	vector3_cross_vector (Q_VECTOR (q1), Q_VECTOR (q2), Q_VECTOR (qtemp));

	vector3_multiply (Q_VECTOR (q1), q2[0], v);
	vector3_sum (v, Q_VECTOR (qtemp), Q_VECTOR (qtemp));

	vector3_multiply (Q_VECTOR (q2), q1[0], v);
	vector3_sum (v, Q_VECTOR (qtemp), Q_VECTOR (q_out));
	q_out[0] = q_out0;

	return q_out;
}

void
quaternion_print (Quaternion q)
{
	vector_print (q, 4);
}

Vector
quaternion_from_axis_angle (Vector3    axis,
                            double     angle,
                            Quaternion q)
{
	Vector v = Q_VECTOR (q);
	double norm = vector3_norm (axis);
	int i;

	q[0] = cos (angle / 2);
	vector_copy (v, axis, 3);

	for (i = 0; i < 3; ++i)
	{
		v[i] *= sin (angle / 2) / norm;
	}

	return q;
}

Vector
quaternion_rotation (Quaternion q,
                     Vector3    v_in,
                     Vector3    v_out)
{
	Quaternion qtemp = {0, v_in[0], v_in[1], v_in[2]};
	Quaternion qbar;
	
	quaternion_conjugate (q, qbar);
	quaternion_product_quaternion (qtemp, qbar, qtemp);
	quaternion_product_quaternion (q, qtemp, qtemp);

	return vector_copy (v_out, Q_VECTOR (qtemp), 3);
}

Vector
quaternion_to_axis_angle (Quaternion  q,
                          Vector3     v_out,
                          double     *angle)
{
	int i;

	*angle = 2 * acos (q[0]);

	if (*angle == 0)
	{
		memset (v_out, 0, sizeof (Vector3));
	}
	else
	{
		double s = sin (*angle / 2);

		for (i = 0; i < 3; ++i)
		{
			v_out[i] = q[i + 1] / s;
		}
	}

	return q;
}
