#ifndef __MATRIX_H
#define __MATRIX_H


#include <stdlib.h>
#include <stdio.h>
#include <string.h>


// #ifdef __cplusplus
// extern "C" {
// #endif

typedef double VectorElement;

/* Generic vector type. Vector3 and Vector6 can implicitely decay to
 template <typename T,int N>
 * this type. Matrix types can be cast to this type for use in vector
 * functions */
typedef VectorElement* Vector;

/* Generic matrix type. Matrix3 and Matrix6 can implicitely decay to
 * this type. */
typedef VectorElement (*Matrix)[];

/* Specific types */
typedef VectorElement Vector3[3];
typedef VectorElement Vector6[6];
typedef VectorElement Matrix3[3][3];
typedef VectorElement Matrix6[6][6];
typedef VectorElement Quaternion[4];

/* Type to pass as parameter a function acting on a vector element */
typedef VectorElement VectorMapFunc (VectorElement);

/* Macro to acces element (I,J) of a generic matrix with number of columns = N_COLS */
#define MATRIX_IJ(MATRIX, I, J, N_COLS) (((Vector)(MATRIX))[(I) * (N_COLS) + (J)])

#define Q_VECTOR(Q) ((Vector) &(Q)[1])

Vector vector_zeros (Vector v, int n);
Vector vector_copy (Vector v_out, Vector v_in, int n);
Vector vector_multiply (Vector v_in, double a, Vector v_out, int n);
Vector vector_sum (Vector v1, Vector v2, Vector v_out, int n);
Vector vector_difference (Vector v1, Vector v2, Vector v_out, int n);
Vector vector_normalize (Vector v_int, Vector v_out, int n);
double vector_dot_product (Vector v1, Vector v2, int n);
double vector_norm (Vector v, int n);
void vector_print (Vector v, int n);

Vector vector3_zeros (Vector3 v);
Vector vector3_multiply (Vector3 v_in,  double a, Vector3 v_out);
Vector vector3_map (Vector3 v_in, VectorMapFunc f, Vector3 v_out);
Matrix vector3_antisymmetric_matrix (Vector3 v_in, Matrix3 m_out);
Matrix vector3_translation_transformation_matrix (Vector3 v_in, Matrix6 m_out);
Vector vector3_cross_vector (Vector3 v1, Vector3 v2, Vector3 v_out);
Vector vector3_times_vector (Vector3 v1, Vector3 v2, Vector3 v_out);
Vector vector3_sum (Vector3 v1, Vector3 v2, Vector3 v_out);
Matrix vector3_tensor_product (Vector3 v1, Vector3 v2, Matrix3 m_out);
double vector3_dot_product (Vector3 v1, Vector3 v2);
Vector vector3_difference (Vector3 v1, Vector3 v2, Vector3 v_out);
Vector vector3_normalize (Vector3 v_in, Vector3 v_out);
double vector3_norm (Vector3 v);
void vector3_print (Vector3 v);

Vector vector6_sum (Vector6 v1, Vector6 v2, Vector6 v_out);

template <typename T,int N>
Matrix matrix_identity (T (*m)[N], int n)
{
	int i;

	memset (m, 0, n * n * sizeof (VectorElement));
	for (i = 0; i < n; ++i)
	{
		MATRIX_IJ (m, i, i, n) = 1;
	}

	return (Matrix) m;
}
template <typename T,int N>
Vector matrix_product_vector_temp (T (*m)[N], Vector v_in, Vector v_out, int rows, int columns, Vector v_temp)
{
	int i, j;

	for (i = 0; i < rows; ++i)
	{
		v_temp[i] = 0;

		for (j = 0; j < columns; ++j)
		{
			v_temp[i] += MATRIX_IJ (m, i, j, columns) * v_in[j];
		}
	}

	return (Vector) memcpy (v_out, v_temp, rows * sizeof (VectorElement));
}

template <typename T,int N>
Vector matrix_product_vector (T (*m)[N], Vector v_in, Vector v_out, int rows, int columns) /* slow */
{
	Vector v_temp = malloc (rows * sizeof (VectorElement));
	matrix_product_vector_temp (m, v_in, v_out, rows, columns, v_temp);
	free (v_temp);
	return (v_out);
}

template <typename T,int N>
Matrix matrix_multiply (T (*m_in)[N], double a, T (*m_out)[N], int rows, int columns)
{
	return (Matrix) vector_multiply ((Vector) m_in, a, (Vector) m_out, rows * columns);
}
template <typename T,int N>
void matrix_print (T (*m)[N], int rows, int columns)
{
	int i, j;

	for (i = 0; i < rows; ++i)
	{
		for (j = 0; j < columns; ++j)
		{
			printf ("\t%f", MATRIX_IJ (m, i, j, columns));
		}
		puts ("");
	}
	puts ("");
}

Matrix matrix3_zeros (Matrix3 m);
Matrix matrix3_transpose (Matrix3 m, Matrix3 mT);
Vector matrix3_product_vector (Matrix3 m_in, Vector3 v_in, Vector3 v_out);
Vector matrix3_product_vector6 (Matrix3 m_in, Vector6 v_in, Vector6 v_out); /* first and second half of Vector6 multiplied separately */
Matrix matrix3_sum (Matrix3 m1, Matrix3 m2, Matrix3 m_out);
Matrix matrix3_multiply (Matrix3 m_in, double a, Matrix3 m_out);
Matrix matrix3_product_matrix3 (Matrix3 m1, Matrix3 m2, Matrix3 m_out);
void matrix3_print (Matrix3 m);

Vector matrixn_product_vector3 (Matrix3 m_in, Vector3 v_in, Vector3 v_out, int n); /* using the first 'n' columns of m_in */

Matrix matrix6_zeros (Matrix6 m);
Matrix matrix6_transpose (Matrix6 m, Matrix6 mT);
Matrix matrix6_sum (Matrix6 m1, Matrix6 m2, Matrix6 m_out);
Matrix matrix6_multiply (Matrix6 m_in, double a, Matrix6 m_out);
Matrix matrix6_get_block3 (Matrix6 m, Matrix3 block, int row, int column);
Matrix matrix6_set_block3 (Matrix6 m, Matrix3 block, int row, int column);
Matrix matrix6_by_blocks3 (Matrix6 m_out, Matrix3 m11, Matrix3 m12, Matrix3 m21, Matrix3 m22);
Vector matrix6_product_vector (Matrix6 m_in, Vector6 v_in, Vector6 v_out);
Matrix matrix6_product_matrix6 (Matrix6 m1, Matrix6 m2, Matrix6 m_out);

Vector quaternion_conjugate (Quaternion q_in, Quaternion q_out);
Vector quaternion_inverse (Quaternion q_in, Quaternion q_out);
Vector quaternion_new (Vector3 axis, double angle);
Vector quaternion_product_quaternion (Quaternion q1, Quaternion q2, Quaternion q_out);
Vector quaternion_from_axis_angle (Vector3 axis, double angle, Quaternion q);
Vector quaternion_rotation (Quaternion q, Vector3 v_in, Vector3 v_out);
Vector quaternion_to_axis_angle (Quaternion q, Vector3 v_out, double *angle);
void quaternion_print (Quaternion q);

// #ifdef __cplusplus
// }
// #endif


#endif
